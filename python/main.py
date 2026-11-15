# FikaBuster - the brain.
# Watches the Modulino Thermo for a cup being placed, asks a cloud vision model what the
# cup and drink are, then runs Newton's Law of Cooling to tell you when to drink.
#   T(t) = T_room + (T_0 - T_room) * e^(-k t)      ->      t = ln((T - T_room) / (T_x - T_room)) / k

import base64
import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request
from collections import deque

from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from arduino.app_bricks.cloud_llm import CloudLLM, CloudModel
from arduino.app_peripherals.camera import Camera
from arduino.app_utils.image import compress_to_jpeg

# ---------------- Tuning ----------------
POLL_S = 0.5            # sensor polling period
JUMP_C = 2.0            # cup detected when the sensor moves more than this...
JUMP_WINDOW_S = 3.0     # ...within this many seconds
BASELINE_S = 30.0       # rolling average window used as room temperature
ZONE_C = 3.0            # perfect zone = target +/- ZONE_C
RUN_LEAD_S = 90         # RUN when the zone closes within this time
SPRINT_RUN_LEAD_S = 20  # same, for espresso
REMINDER_S = 300        # warning beep 5 min after DRINK if the cup was not picked up
COLD_DANGER_S = 180     # cold drinks: danger zone when the bound is reached within this time
SETTLE_S = 60           # the chamber under the cup keeps heating up for a while after placing
LIFT_C = 1.5            # sudden move in the opposite direction = cup lifted
LIFT_TIMEOUT_S = 120    # lifted for this long = cup gone, session ends
TARGET_ADJ_MAX = 5.0    # knob adjusts the target by +/- this much
AUTO_ADD_CONFIDENCE = 0.6  # an unknown drink seen on camera is added (via Exa) only above this AI confidence
AUTO_ADD_TIMEOUT_S = 25    # Exa lookup while the cup is being analysed: don't keep the spinner forever

# heat_transfer_efficiency: how much of the liquid-room difference reaches the sensor
# cooling_rate_k: Newton cooling constant, per minute
MATERIALS = {
    "ceramic": {"heat_transfer_efficiency": 0.6, "cooling_rate_k": 0.015},
    "paper":   {"heat_transfer_efficiency": 0.9, "cooling_rate_k": 0.035},
    "glass":   {"heat_transfer_efficiency": 0.7, "cooling_rate_k": 0.020},
    "metal":   {"heat_transfer_efficiency": 0.85, "cooling_rate_k": 0.030},
    "plastic": {"heat_transfer_efficiency": 0.8, "cooling_rate_k": 0.030},
    "thermos": {"heat_transfer_efficiency": 0.05, "cooling_rate_k": 0.004},  # open lid, heat leaves only from the top
}
FILL_K = {"full": 1.0, "half": 1.5, "nearly_empty": 2.2}  # less liquid = less thermal mass = faster cooling
ESPRESSO_K_MULT = 8.0  # 30 ml has almost no thermal mass: Sprint Mode

# Default drink database: copied to data/drinks.json on first start, then edited from the web UI.
# target: ideal temperature (hot) or upper bound (cold); fresh: assumed start temp when the sensor is bypassed
DEFAULT_DRINKS = {
    "coffee_black":  {"label": "Coffee (black)", "target": 60, "fresh": 80},
    "coffee_milk":   {"label": "Coffee (latte/milk)", "target": 55, "fresh": 70},
    "espresso":      {"label": "Espresso", "target": 60, "fresh": 70, "sprint": True},
    "green_tea":     {"label": "Green tea", "target": 70, "fresh": 80},
    "black_tea":     {"label": "Black tea", "target": 65, "fresh": 90},
    "herbal_tea":    {"label": "Herbal tea", "target": 65, "fresh": 90},
    "hot_chocolate": {"label": "Hot chocolate", "target": 55, "fresh": 70},
    "cold_drink":    {"label": "Iced beverage", "target": 10, "fresh": 4, "cold": True},
    "beer":          {"label": "Beer", "target": 10, "fresh": 5, "cold": True, "afterhours": True},
}

VISION_SYSTEM = ("You are the vision module of FikaBuster, a smart coaster. You see a photo, taken from above at an angle, "
                 "of a beverage standing on the coaster. Answer ONLY with a compact JSON object, no prose, no markdown.")
VISION_PROMPT = """Look at this beverage and its container. Return a JSON object with these fields:
- "drink": one of {drinks}, or "other" if the beverage is clearly none of these.
- "new_drink": only when "drink" is "other": the common English name of the beverage, short and specific (e.g. "Chai latte"); otherwise null.
- "material": one of "ceramic", "glass", "paper", "metal", "plastic", "thermos" (double-walled insulated tumbler or bottle such as Yeti or Stanley).
- "lid": "on", "off" or "none" (is a lid covering the drink?).
- "fill": "full", "half" or "nearly_empty" (look inside the cup: how much liquid is left?).
- "surface": "crema", "milk_foam", "flat", "bubbles" or "unknown" (liquid surface texture).
- "cup_size": "small", "medium" or "large" (cup size relative to the frame).
- "confidence": number between 0 and 1.
If no beverage is clearly visible, give your best guess with a low confidence."""

APP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
PREFS_PATH = os.path.join(APP_DIR, "data", "preferences.json")
DRINKS_PATH = os.path.join(APP_DIR, "data", "drinks.json")

# Exa /answer: searches the web and returns the answer in this shape, with citations
EXA_URL = "https://api.exa.ai/answer"
EXA_SCHEMA = {
    "type": "object",
    "properties": {
        "cold": {"type": "boolean", "description": "true if the drink is normally served cold or iced"},
        "target": {"type": "number", "description": "ideal drinking temperature in °C; for a cold drink the warmest it still tastes good"},
        "fresh": {"type": "number", "description": "typical temperature in °C right after it is served or poured"},
    },
    "required": ["cold", "target", "fresh"],
}

# ---------------- Peripherals ----------------
# Best first. The USB webcam only offers 640x480 uncompressed; higher resolutions need MJPG.
# 640x480 with the default codec is the known-good fallback.
CAMERA_MODES = [((1024, 768), "MJPG"), ((640, 480), "")]


def try_camera(resolution, codec):
    """Started camera in this mode if it really delivers frames, else None."""
    cam = Camera(resolution=resolution, fps=15, codec=codec)  # first camera found, USB first
    cam.start()
    for _ in range(10):
        frame = cam.capture()
        if frame is not None:
            print(f"[camera] opened at {frame.shape[1]}x{frame.shape[0]}")
            return cam
        time.sleep(0.2)
    print(f"[camera] no frames at {resolution[0]}x{resolution[1]}")
    cam.stop()
    return None


def open_camera(attempts):
    for attempt in range(attempts):  # the USB camera can take a moment to show up after a restart
        for resolution, codec in CAMERA_MODES:
            try:
                cam = try_camera(resolution, codec)
                if cam is not None:
                    return cam
            except Exception as e:
                print(f"[camera] open attempt {attempt + 1} at {resolution[0]}x{resolution[1]} failed: {e}")
        if attempt + 1 < attempts:
            time.sleep(2)
    return None


camera = open_camera(3)

def model_for_key(key):
    if key.startswith("sk-ant-"):
        return CloudModel.ANTHROPIC_CLAUDE
    if key.startswith("sk-"):
        return CloudModel.OPENAI_GPT
    return CloudModel.GOOGLE_GEMINI  # Gemini keys start with "AIza"


llm = None
api_key = os.getenv("API_KEY", "").strip()  # from the Cloud LLM Brick Configuration
if api_key:
    try:
        llm = CloudLLM(api_key=api_key, model=model_for_key(api_key), system_prompt=VISION_SYSTEM, timeout=30)
        print(f"[ai] cloud vision ready ({model_for_key(api_key)})")
    except Exception as e:
        print(f"[ai] FAILED to init CloudLLM: {e}")
else:
    print("[ai] no API_KEY in the Cloud LLM Brick Configuration: using default cup/drink guesses")
print(f"[exa] {'drink lookup ready' if os.getenv('EXA_API_KEY', '').strip() else 'no EXA_API_KEY in the Exa Brick Configuration'}")

# ---------------- State ----------------
lock = threading.RLock()
samples = deque(maxlen=int(BASELINE_S / POLL_S) + 20)   # (time, sensor temp)
history = deque(maxlen=300)                              # for the web chart
events = deque(maxlen=30)
knob_last = None             # last knob counters from the sketch
session = None
pending = None               # analysis in progress
generation = 0               # bumps on reset so a late AI answer is discarded
overlay = None               # (display spec, until)
last_display = None
last_room = None             # (room temp, time) of the previous cup
snapshot = {"id": 0, "img": None}
hw = {"ok": False, "temp": None, "hum": None, "camera": camera is not None, "ai": llm is not None}


def log(msg):
    print(msg)
    events.appendleft({"ts": int(time.time() * 1000), "msg": msg})


# ---------------- Preferences ----------------
def load_prefs():
    try:
        with open(PREFS_PATH) as f:
            return json.load(f)
    except Exception:
        return {}


def save_prefs(p):
    os.makedirs(os.path.dirname(PREFS_PATH), exist_ok=True)
    with open(PREFS_PATH, "w") as f:
        json.dump(p, f, indent=2)


prefs = load_prefs()


def pref_ok(cold, value):
    return -5 <= value <= 25 if cold else 35 <= value <= 95


def target_for(drink, d):
    value = prefs.get(f"user_{drink}_pref")
    if isinstance(value, (int, float)) and pref_ok(d["cold"], value):
        return float(value)
    return float(d["target"])


# ---------------- Drink database ----------------
def clean_drink(v):
    """Validated copy of a drink entry; raises ValueError with a user-facing message."""
    label = str(v.get("label") or "").strip()[:40]
    if not label:
        raise ValueError("Give the drink a name")
    cold = bool(v.get("cold"))
    try:
        target, fresh = float(v.get("target")), float(v.get("fresh"))
    except (TypeError, ValueError):
        raise ValueError("Temperatures must be numbers")
    if not pref_ok(cold, target):
        raise ValueError("Max temperature must be -5…25 °C" if cold else "Target must be 35…95 °C")
    if cold and not -5 <= fresh < target:
        raise ValueError("Start temperature must be -5 °C or more and below the max temperature")
    if not cold and not target < fresh <= 100:
        raise ValueError("Start temperature must be above the target and at most 100 °C")
    return {"label": label, "target": round(target, 1), "fresh": round(fresh, 1), "cold": cold,
            "sprint": bool(v.get("sprint")) and not cold, "afterhours": bool(v.get("afterhours")) and cold}


def has_both_kinds(drinks):
    kinds = {d["cold"] for d in drinks.values()}
    return kinds == {True, False}


def load_drinks():
    try:
        with open(DRINKS_PATH) as f:
            drinks = {str(k): clean_drink(v) for k, v in json.load(f).items()}
        if has_both_kinds(drinks):
            return drinks
        print(f"[drinks] {DRINKS_PATH} needs at least one hot and one cold drink: using the defaults")
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"[drinks] {DRINKS_PATH} unreadable, using the defaults: {e}")
    return {k: clean_drink(v) for k, v in DEFAULT_DRINKS.items()}


drinks_rev = 0  # bumps on every change so the web UI reloads its drink list


def save_drinks():
    global drinks_rev
    drinks_rev += 1
    os.makedirs(os.path.dirname(DRINKS_PATH), exist_ok=True)
    with open(DRINKS_PATH, "w") as f:
        json.dump(DRINKS, f, indent=2)


DRINKS = load_drinks()
save_drinks()  # writes the defaults on first start and drops fields older versions stored


def default_drink(cold):
    """Fallback when the picture doesn't match the temperature: the original default, else the first of that kind."""
    preferred = "cold_drink" if cold else "coffee_black"
    if preferred in DRINKS and DRINKS[preferred]["cold"] == cold:
        return preferred
    return next(k for k, d in DRINKS.items() if d["cold"] == cold)


def new_drink_id(label):
    base = re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_") or "drink"
    key, n = base, 2
    while key in DRINKS:
        key, n = f"{base}_{n}", n + 1
    return key


# ---------------- Exa drink lookup ----------------
def exa_key():
    return os.getenv("EXA_API_KEY", "").strip()  # from the Exa brick's Brick Configuration


def exa_lookup(name, timeout=60):
    """Ask Exa about a drink; returns form-ready values (clamped to the valid ranges) and the sources."""
    key = exa_key()
    if not key:
        raise ValueError("Set EXA_API_KEY in the Exa brick's Brick Configuration and restart the app")
    body = {"query": f"{name} (beverage): ideal drinking temperature and typical serving temperature in degrees Celsius, "
                     f"and is it served hot or cold?",
            "outputSchema": EXA_SCHEMA}
    req = urllib.request.Request(EXA_URL, data=json.dumps(body).encode(), method="POST",
                                 headers={"x-api-key": key, "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            reply = json.load(r)
    except urllib.error.HTTPError as e:
        raise ValueError("Exa rejected the API key" if e.code in (401, 403) else f"Exa error {e.code}")
    except urllib.error.URLError as e:
        raise ValueError(f"Exa not reachable: {e.reason}")
    a = reply.get("answer")
    if not isinstance(a, dict):
        raise ValueError("Exa returned no structured answer")
    cold = bool(a.get("cold"))
    try:
        target, fresh = round(float(a.get("target"))), round(float(a.get("fresh")))
    except (TypeError, ValueError):
        raise ValueError("Exa found no temperatures for this drink")
    if cold:  # clamp so the form can be saved as is
        target = min(25, max(-4, target))
        fresh = min(target - 1, max(-5, fresh))
    else:
        target = min(95, max(35, target))
        fresh = min(100, max(target + 1, fresh))
    sources = [{"title": c.get("title") or c.get("url"), "url": c.get("url")} for c in reply.get("citations", []) if str(c.get("url", "")).startswith("http")][:3]
    return {"cold": cold, "target": target, "fresh": fresh, "sources": sources}


auto_add_failed = set()  # names Exa couldn't answer this run: don't ask again for every cup


def drink_by_label(label):
    want = label.strip().lower()
    return next((k for k, d in DRINKS.items() if d["label"].lower() == want), None)


def auto_add_drink(name):
    """Drink id for a beverage the camera saw but the database lacks: looked up with Exa and saved. None on failure."""
    name = name.strip()[:40]
    name = name[:1].upper() + name[1:]
    with lock:
        existing = drink_by_label(name)
    if existing:
        return existing
    if not exa_key() or name.lower() in auto_add_failed:
        return None
    log(f"New drink on camera: {name}, asking Exa")
    try:
        found = exa_lookup(name, timeout=AUTO_ADD_TIMEOUT_S)  # outside the lock: takes a few seconds
        d = clean_drink({"label": name, **found})
    except Exception as e:  # ValueError from Exa/validation, or a socket timeout
        auto_add_failed.add(name.lower())
        log(f"Could not add {name}: {e}")
        return None
    with lock:
        key = drink_by_label(name)  # added from the web UI meanwhile?
        if key is None:
            key = new_drink_id(name)
            DRINKS[key] = d
            save_drinks()
            log(f"Drink added automatically: {name} ({'cold' if d['cold'] else 'hot'}, "
                f"target {d['target']} C, start {d['fresh']} C)")
    return key


def vision_prompt():
    with lock:
        drinks = ", ".join(f'"{k}" ({d["label"]})' for k, d in DRINKS.items())
    return VISION_PROMPT.format(drinks=drinks)


# ---------------- Hardware helpers ----------------
def show(spec):
    global last_display
    if spec == last_display:
        return
    try:
        Bridge.call("set_display", spec)
        last_display = spec
    except Exception as e:
        print(f"[bridge] display error: {e}")


def play(sound):
    try:
        Bridge.call("play", sound)
    except Exception as e:
        print(f"[bridge] play error: {e}")


def set_overlay(spec, seconds):
    global overlay
    overlay = (spec, time.time() + seconds)


def fmt_time(s):
    if s == math.inf:
        return "--"
    s = max(0, int(s))
    if s < 60:
        return f"{s}S"
    return f"{int(round(s / 60))}M"


# ---------------- Sensor analysis ----------------
def window(now):
    return [t for (ts, t) in samples if ts >= now - JUMP_WINDOW_S]


def estimate_room(now):
    old = [t for (ts, t) in samples if now - BASELINE_S <= ts < now - JUMP_WINDOW_S]
    room = sum(old) / len(old) if old else (samples[0][1] if samples else 22.0)
    # The chamber may still be warm/cold from the previous cup: room temperature doesn't move that fast
    if last_room and now - last_room[1] < 1200 and abs(room - last_room[0]) > 1.0:
        room = last_room[0]
    return room


# ---------------- Vision ----------------
def snap():
    global camera
    if camera is None:
        camera = open_camera(1)  # plugged in after start: snapshots work, the live stream needs a restart
        hw["camera"] = camera is not None
        if camera is None:
            return None
    for _ in range(5):
        try:
            frame = camera.capture()
            if frame is not None:
                jpg = compress_to_jpeg(frame=frame, quality=85)
                if jpg is not None:
                    return jpg.tobytes()
        except Exception as e:
            print(f"[camera] capture error: {e}")
        time.sleep(0.2)
    return None


def pick(value, options, default):
    value = str(value or "").strip().lower().replace(" ", "_").replace("-", "_")
    return value if value in options else default


def ask_ai(img):
    if llm is None or img is None:
        return None
    try:
        reply = llm.chat(vision_prompt(), images=[img])
        m = re.search(r"\{.*\}", reply, re.S)
        data = json.loads(m.group(0)) if m else {}
        drink = pick(data.get("drink"), DRINKS, None)
        new_drink = str(data.get("new_drink") or "").strip()
        return {
            "drink": drink,
            "new_drink": new_drink if drink is None and new_drink.lower() not in ("", "null", "none", "unknown") else None,
            "material": pick(data.get("material"), MATERIALS, "ceramic"),
            "lid": pick(data.get("lid"), ("on", "off", "none"), "none"),
            "fill": pick(data.get("fill"), FILL_K, "full"),
            "surface": pick(data.get("surface"), ("crema", "milk_foam", "flat", "bubbles", "unknown"), "unknown"),
            "cup_size": pick(data.get("cup_size"), ("small", "medium", "large"), "medium"),
            "confidence": data.get("confidence"),
        }
    except Exception as e:
        log(f"AI vision failed: {e}")
        return None


# ---------------- Session (one cup) ----------------
class Session:
    def __init__(self, info, ai):
        self.t_start = info["t_start"]
        self.room = info["room"]
        self.ai = ai
        a = ai or {}
        sensor = hw["temp"] if hw["temp"] is not None else self.room
        direction = info["direction"]
        drink = a.get("drink") if a.get("drink") in DRINKS else None  # may have been removed during the analysis
        if direction is None:  # manual trigger: decide from the picture, then from the sensor
            if drink:
                direction = "cold" if DRINKS[drink]["cold"] else "hot"
            else:
                direction = "cold" if sensor < self.room - 1 else "hot"
        # The temperature direction wins over the picture (Ice-Fika protocol)
        if drink is None or DRINKS[drink]["cold"] != (direction == "cold"):
            drink = default_drink(direction == "cold")
        self.drink = drink
        self.info = d = dict(DRINKS[drink])  # a copy: editing the database doesn't change a running session
        self.cold = direction == "cold"
        self.material = a.get("material", "glass" if self.cold else "ceramic")
        self.fill = a.get("fill", "full")
        self.lid = a.get("lid", "none")
        self.afterhours = bool(d.get("afterhours"))
        self.sprint = bool(d.get("sprint")) or (a.get("surface") == "crema" and a.get("cup_size") == "small")
        m = MATERIALS[self.material]
        self.eff = m["heat_transfer_efficiency"]
        self.k = m["cooling_rate_k"] * FILL_K[self.fill] * (ESPRESSO_K_MULT if self.sprint else 1.0)
        self.thermos = self.material == "thermos"
        self.mode = ("thermos_lid" if self.lid == "on" else "thermos_open") if self.thermos else ("cold" if self.cold else "hot")
        self.target_base = target_for(drink, d)
        self.target_adj = 0.0
        self.time_offset_s = 0.0
        self.phase = None
        self.drink_since = None
        self.picked_up = False
        self.lifted_since = None
        self.peak_delta = abs(sensor - self.room)
        self.near_room_since = None
        self.last_tick = 0
        self.knob = None  # (what was adjusted last, when): lets the web UI show the change
        if self.thermos:
            self.t0 = float(d["fresh"])  # the bottom sensor can't feel a vacuum flask: pure model
        else:
            self.t0 = self.implied_t0(sensor, time.time())

    # --- physics ---
    def live_estimate(self, sensor):
        est = (sensor - self.room) / self.eff + self.room
        return max(-2.0, min(100.0, est))

    def elapsed_min(self, now):
        return max(0.0, now - self.t_start + self.time_offset_s) / 60.0

    def implied_t0(self, sensor, now):
        return self.room + (self.live_estimate(sensor) - self.room) * math.exp(self.k * self.elapsed_min(now))

    def model(self, now):
        return self.room + (self.t0 - self.room) * math.exp(-self.k * self.elapsed_min(now))

    def time_to(self, temp_x, now):
        """Seconds until the liquid reaches temp_x (0 if already past, inf if never)."""
        a, b = self.model(now) - self.room, temp_x - self.room
        if a == 0 or b == 0 or (a > 0) != (b > 0):
            return math.inf
        if abs(b) >= abs(a):
            return 0.0
        return math.log(a / b) / self.k * 60.0

    def target(self):
        return self.target_base + self.target_adj

    # --- live tracking ---
    def track(self, now, sensor):
        if self.thermos or sensor is None:
            return
        win = window(now)
        rise = sensor - min(win) if win else 0
        drop = max(win) - sensor if win else 0
        # Lifting a hot cup makes the sensor drop, lifting a cold one makes it rise
        away, back = (rise, drop) if self.cold else (drop, rise)
        if self.lifted_since is None and away >= LIFT_C:
            self.lifted_since = now
            self.picked_up = True
            log("Cup lifted")
        elif self.lifted_since is not None and back >= LIFT_C:
            self.lifted_since = None
            log("Cup back on the coaster")
        self.peak_delta = max(self.peak_delta, abs(sensor - self.room))

        if self.lifted_since is None:
            implied = self.implied_t0(sensor, now)
            if now - self.t_start < SETTLE_S:
                # The chamber is still heating/cooling: keep the most extreme reading
                self.t0 = min(self.t0, implied) if self.cold else max(self.t0, implied)
            elif abs(self.live_estimate(sensor) - self.model(now)) < 15:
                self.t0 += 0.05 * (implied - self.t0)

    def finished(self, now, sensor):
        if self.thermos or sensor is None:
            return None
        if self.lifted_since is not None and now - self.lifted_since > LIFT_TIMEOUT_S:
            return "cup removed"
        if now - self.t_start > SETTLE_S and abs(sensor - self.room) < max(1.0, 0.2 * self.peak_delta):
            self.near_room_since = self.near_room_since or now
            if now - self.near_room_since > 20:
                return "sensor back at room temperature"
        else:
            self.near_room_since = None
        return None

    # --- state machine ---
    def compute_phase(self, now):
        if self.mode == "thermos_lid":
            return "THERMOS", math.inf
        T, tgt = self.model(now), self.target()
        if self.cold:
            if T >= tgt:
                return "WARM", 0.0
            left = self.time_to(tgt, now)
            return ("DANGER" if T >= tgt - ZONE_C or left <= COLD_DANGER_S else "CHILL"), left
        hi, lo = tgt + ZONE_C, tgt - ZONE_C
        if T > hi:
            return "WAIT", self.time_to(hi, now)
        if T >= lo:
            if self.drink_since is None:
                self.drink_since = now
            left = self.time_to(lo, now)
            lead = SPRINT_RUN_LEAD_S if self.sprint else RUN_LEAD_S
            reminder = not self.picked_up and now - self.drink_since >= REMINDER_S
            return ("RUN" if left <= lead or reminder else "DRINK"), left
        return "SAD", 0.0

    def update(self, now):
        new, remaining = self.compute_phase(now)
        old = self.phase
        if new != old:
            self.phase = new
            if new not in ("DRINK", "RUN"):
                self.drink_since = None
            log(f"{old or 'START'} -> {new}")
            if new == "DRINK" and old != "RUN":
                play("happy")
            elif new == "RUN":
                play("warn")
            elif new == "SAD":
                play("womp")
            elif new == "THERMOS":
                play("robot")
            elif new == "DANGER":
                play("frantic" if self.afterhours else "warn")
            elif new == "WARM":
                play("trombone" if self.afterhours else "womp")
        if new == "DANGER" and self.afterhours and now - self.last_tick >= 2:
            self.last_tick = now
            play("tick")
        return remaining

    def display(self, now, remaining):
        p = self.phase
        if p == "WAIT":
            return "clock" if self.sprint else "text:" + fmt_time(remaining)
        if p == "DRINK":
            return "icon:smile" if int(now / 3) % 2 == 0 else "text:DRINK"
        if p == "RUN":
            return "blinktext:RUN"
        if p == "SAD":
            return "icon:sad"
        if p == "THERMOS":
            return "text:THERMOS DETECTED. SEE YOU IN 2028"
        if p == "CHILL":
            return ("text:CHILL " if self.afterhours else "text:") + fmt_time(remaining)
        if p == "DANGER":
            return "blinktext:DRINK!" if self.afterhours else "blink:warn"
        if p == "WARM":
            return "icon:melt" if self.afterhours else "icon:sad"
        return "icon:cup"


# ---------------- Actions ----------------
def start_analysis(direction, manual=False):
    global pending
    with lock:
        if pending is not None:
            return False
        end_session(None)
        now = time.time()
        pending = {"t_start": now, "room": estimate_room(now), "direction": direction, "gen": generation}
        kind = "manual" if manual else f"{direction} cup"
        log(f"Cup detected ({kind}), room temperature {pending['room']:.1f} C")
    play("ack")
    threading.Thread(target=analyze_worker, args=(pending,), daemon=True).start()
    return True


def analyze_worker(info):
    global session, pending
    img = snap()
    if img is not None:
        with lock:
            snapshot["id"] += 1
            snapshot["img"] = "data:image/jpeg;base64," + base64.b64encode(img).decode()
    else:
        log("No camera picture: using default guesses")
    ai = ask_ai(img)
    if ai and ai["new_drink"]:
        try:
            confident = float(ai.get("confidence") or 0) >= AUTO_ADD_CONFIDENCE
        except (TypeError, ValueError):
            confident = False
        if confident:
            ai["drink"] = auto_add_drink(ai["new_drink"])  # the cup's timeline still starts at t_start
    with lock:
        pending = None
        if info["gen"] != generation:
            return
        session = Session(info, ai)
        s = session
        log(f"AI: {json.dumps(ai) if ai else 'n/a'} -> {s.info['label']} in {s.material} ({s.mode}), "
            f"liquid {s.t0:.1f} C, k {s.k:.3f}/min, target {s.target():.1f} C")
        if s.afterhours:
            play("party")
            set_overlay("icon:beer", 3)
        elif s.cold:
            set_overlay("icon:snow", 3)
        elif s.sprint:
            set_overlay("text:SPRINT", 3)


def end_session(reason):
    global session, last_room, generation, pending
    with lock:
        if session is not None:
            last_room = (session.room, time.time())
            if reason:
                log(f"Session ended: {reason}")
        session = None
        if reason:
            generation += 1
            pending = None


def save_preference():
    with lock:
        s = session
        if s is None or s.mode == "thermos_lid":
            return None
        now = time.time()
        value = round(s.model(now), 1)
        if s.cold:
            value = round(value + ZONE_C + 1, 1)  # "perfect right now" -> the upper bound sits a bit above
        if not pref_ok(s.cold, value):
            log(f"Not saving {value} C as a preference for {s.info['label']}: out of range")
            play("womp")
            return None
        prefs[f"user_{s.drink}_pref"] = value
        save_prefs(prefs)
        s.target_base, s.target_adj = value, 0.0
        if not s.cold:
            s.phase, s.drink_since = "DRINK", now  # we're at the perfect temp by definition: no extra beeps
        log(f"Saved preference user_{s.drink}_pref = {value} C")
    play("saved")
    set_overlay("icon:heart", 2.5)
    return value


def handle_knob(free, held, clicks, doubles, longs):
    with lock:
        s = session
        if longs:
            end_session("long press")
            show_now()
            return
        if doubles:
            if s is not None:
                save_preference()
            return
        if clicks:
            if s is None:
                if pending is None:
                    start_analysis(None, manual=True)
            else:
                s.picked_up = True  # acknowledge: silences the 5-minute reminder
                set_overlay(f"text:{round(s.model(time.time()))}^", 2)
        if s is None:
            return
        if free:
            s.target_adj = max(-TARGET_ADJ_MAX, min(TARGET_ADJ_MAX, s.target_adj + free))
            s.knob = ("target", time.time())
            set_overlay(f"text:{round(s.target())}^", 1.5)
        if held and s.phase in ("WAIT", "CHILL"):
            s.time_offset_s -= held * 60  # clockwise adds a minute to the countdown
            s.knob = ("time", time.time())
            remaining = s.time_to(s.target() + (0 if s.cold else ZONE_C), time.time())
            set_overlay("text:" + fmt_time(remaining), 1.5)


def show_now():
    global overlay
    now = time.time()
    if overlay and now >= overlay[1]:
        overlay = None
    if pending is not None:
        spec = "spin"
    elif session is not None:
        spec = session.display(now, session.update(now))
    else:
        spec = "icon:cup"
    show(overlay[0] if overlay else spec)


# ---------------- Main loop ----------------
def loop():
    global knob_last
    now = time.time()
    try:
        ok, temp, hum, kf, kh, kc, kd, kl = Bridge.call("read_sensors").split(",")
        if (ok == "1") != hw["ok"] or knob_last is None:  # first read or a change
            print(f"[sensor] Modulino Thermo {'online' if ok == '1' else 'offline'}")
        hw["ok"] = ok == "1"
        hw["temp"] = float(temp) if hw["ok"] else None
        hw["hum"] = float(hum) if hw["ok"] else None
        counters = [int(kf), int(kh), int(kc), int(kd), int(kl)]
        if knob_last is not None:
            diff = [c - l for c, l in zip(counters, knob_last)]
            if any(diff):
                handle_knob(*diff)
        knob_last = counters
    except Exception as e:
        print(f"[bridge] read error: {e}")
        time.sleep(POLL_S)
        return

    with lock:
        sensor = hw["temp"]
        if sensor is not None:
            samples.append((now, sensor))
            if session is None and pending is None and samples[0][0] <= now - JUMP_WINDOW_S:
                win = window(now)
                room = estimate_room(now)
                # The extra room check ignores a still-warm chamber cooling down after a cup left
                if sensor - min(win) > JUMP_C and sensor > room + 1:
                    start_analysis("hot")
                elif sensor - max(win) < -JUMP_C and sensor < room - 1:
                    start_analysis("cold")
            elif session is not None:
                session.track(now, sensor)
                reason = session.finished(now, sensor)
                if reason:
                    end_session(reason)
        if not history or now - history[-1]["ts"] / 1000 >= 2:
            history.append({"ts": int(now * 1000), "sensor": sensor,
                            "liquid": round(session.model(now), 2) if session else None})
        show_now()
    time.sleep(POLL_S)


# ---------------- Web UI ----------------
def api_state():
    with lock:
        now = time.time()
        out = {"hw": hw, "phase": "ANALYZING" if pending else (session.phase if session else "IDLE"),
               "display": last_display, "room": round(estimate_room(now), 2) if samples else None,
               "snapshot_id": snapshot["id"], "drinks_rev": drinks_rev, "events": list(events), "history": list(history),
               "prefs": prefs, "session": None}
        s = session
        if s:
            _, remaining = s.compute_phase(now)
            out["room"] = round(s.room, 2)
            out["session"] = {
                "drink": s.info["label"], "material": s.material, "fill": s.fill, "lid": s.lid,
                "mode": s.mode, "sprint": s.sprint, "afterhours": s.afterhours, "ai": s.ai,
                "liquid": round(s.model(now), 1),
                "live": round(s.live_estimate(hw["temp"]), 1) if hw["temp"] is not None and not s.thermos else None,
                "target": round(s.target(), 1), "zone": ZONE_C, "k": round(s.k, 4),
                "adj": round(s.target_adj, 1), "adj_max": TARGET_ADJ_MAX, "time_adj": round(-s.time_offset_s / 60),
                "knob": {"kind": s.knob[0], "age": round(now - s.knob[1], 2)} if s.knob else None,
                "remaining": None if remaining == math.inf else int(remaining),
                "elapsed": int(now - s.t_start),
            }
        return out


def api_analyze():
    return {"ok": start_analysis(None, manual=True)}


def api_reset():
    end_session("reset from web")
    return {"ok": True}


def api_save():
    v = save_preference()
    return {"ok": v is not None, "value": v}


def api_beep():
    play("ack")
    return {"ok": True}


def api_drinks():
    with lock:
        return {"drinks": [{"id": k, **d, "pref": target_for(k, d) if target_for(k, d) != d["target"] else None}
                           for k, d in DRINKS.items()]}


def api_drink_save(body: dict):
    with lock:
        try:
            d = clean_drink(body)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        key = str(body.get("id") or "")
        if key in DRINKS:
            if not has_both_kinds({**DRINKS, key: d}):
                return {"ok": False, "error": "Keep at least one hot and one cold drink"}
            if DRINKS[key]["cold"] != d["cold"] and prefs.pop(f"user_{key}_pref", None) is not None:
                save_prefs(prefs)  # a hot favourite makes no sense for a cold drink
            log(f"Drink updated: {d['label']}")
        else:
            key = new_drink_id(d["label"])
            log(f"Drink added: {d['label']}")
        DRINKS[key] = d
        save_drinks()
        return {"ok": True, "id": key}


def api_drink_delete(body: dict):
    with lock:
        key = str(body.get("id") or "")
        if key not in DRINKS:
            return {"ok": False, "error": "No such drink"}
        rest = {k: d for k, d in DRINKS.items() if k != key}
        if not has_both_kinds(rest):
            return {"ok": False, "error": "Keep at least one hot and one cold drink"}
        label = DRINKS.pop(key)["label"]
        if prefs.pop(f"user_{key}_pref", None) is not None:
            save_prefs(prefs)
        save_drinks()
        log(f"Drink removed: {label}")
        return {"ok": True}


def api_drinks_reset():
    with lock:
        DRINKS.clear()
        DRINKS.update({k: clean_drink(v) for k, v in DEFAULT_DRINKS.items()})
        save_drinks()
        log("Drinks restored to the defaults")
        return {"ok": True}


def api_drink_lookup(body: dict):
    name = str(body.get("label") or "").strip()[:40]
    if not name:
        return {"ok": False, "error": "Type a drink name first"}
    try:
        found = exa_lookup(name)  # outside the lock: the web request takes a few seconds
    except ValueError as e:
        return {"ok": False, "error": str(e)}
    log(f"Exa lookup: {name} -> {'cold' if found['cold'] else 'hot'}, target {found['target']} C, start {found['fresh']} C")
    return {"ok": True, **found}


ui = WebUI()
ui.expose_api("GET", "/api/state", api_state)
ui.expose_api("GET", "/api/snapshot", lambda: {"id": snapshot["id"], "img": snapshot["img"]})
ui.expose_api("POST", "/api/analyze", api_analyze)
ui.expose_api("POST", "/api/reset", api_reset)
ui.expose_api("POST", "/api/save", api_save)
ui.expose_api("POST", "/api/beep", api_beep)
ui.expose_api("GET", "/api/drinks", api_drinks)
ui.expose_api("POST", "/api/drinks", api_drink_save)
ui.expose_api("POST", "/api/drinks/delete", api_drink_delete)
ui.expose_api("POST", "/api/drinks/reset", api_drinks_reset)
ui.expose_api("POST", "/api/drinks/lookup", api_drink_lookup)
if camera is not None:
    ui.expose_camera("/stream", camera)

App.run(user_loop=loop)
