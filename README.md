# ☕ FikaBuster

A predictive coaster: it recognises your cup and drink, models how it cools (Newton's Law of Cooling) and beeps when the drink reaches its perfect temperature.

## Hardware
- Arduino UNO Q, USB camera looking down at the coaster
- Modulino Thermo (under the cup), Modulino Knob, Modulino Buzzer, built-in LED matrix

## Setup
Set `API_KEY` in the Cloud LLM brick's Brick Configuration in App Lab and restart the app. The provider is picked from the key: Gemini (`AIza...`), OpenAI (`sk-...`) or Anthropic (`sk-ant-...`). Without a key the app still works, assuming a black coffee in a ceramic mug (or an iced drink in a glass).

## How it works
1. The Thermo is read continuously; its rolling average is the room temperature.
2. A jump of more than 2 °C within 3 s means a cup was placed (up = hot, down = cold "Ice-Fika" mode). A knob click triggers it manually, e.g. for a thermos.
3. The camera takes a snapshot, the cloud vision model returns drink, material, lid, fill level, surface and cup size.
4. Liquid temperature = `(sensor - room) / heat_transfer_efficiency + room`; `k` comes from the material, the fill level and Espresso Sprint Mode.
5. `T(t) = T_room + (T_0 - T_room)·e^(-kt)` is solved for the time to the perfect zone (target ±3 °C). The live sensor keeps correcting the model.

| State | LED | Buzzer |
|---|---|---|
| Analyzing | spinner | short beep |
| WAIT | minutes left (fast clock for espresso) | silent |
| DRINK | smiley / "DRINK" | 2 happy beeps |
| RUN | blinking "RUN" (zone closing, or 5 min without pickup) | fast warning beeps |
| SAD | sad face | womp womp |
| Thermos, lid on | "THERMOS DETECTED. SEE YOU IN 2028" | robot sound |
| AfterHours (beer) | beer mug → "CHILL 12M" → "DRINK!" → melting face | party beep → frantic ticking → sad trombone |
| Iced drink | snowflake → minutes → warning sign → sad face | |

Default targets: black coffee 60 °C, latte 55 °C, espresso 60 °C, green tea 70 °C, black/herbal tea 65 °C, hot chocolate 55 °C, iced drinks and beer 10 °C upper bound.

## Drinks
The drink list is a small database in `data/drinks.json`, created from the defaults on first start. Add, edit or remove drinks in the **Drinks** card of the web UI: name, hot or cold, target (or max) temperature, start temperature, Sprint/AfterHours mode. At least one hot and one cold drink must stay; **Restore defaults** brings the original list back.

**Fill in with Exa**: set `EXA_API_KEY` in the Brick Configuration of the app's own **Exa** brick (`bricks/exa/`) and restart the app. In the drink form, type a name and press **🔎 Fill in with Exa**: Exa searches the web and returns hot/cold, the ideal and serving temperatures as structured data, with source links. Review the values, then save.

## Knob
- Turn: target ±1 °C (max ±5)
- Hold + turn: ±1 minute on the countdown
- Click: start an analysis (no cup) / show the current temperature and silence the reminder
- Double-click: save the current temperature as your preference for this drink (`data/preferences.json`)
- Long press: end the session

## Calibration
The `MATERIALS` table in `python/main.py` holds the heat-transfer efficiency and cooling constant per material. They are starting values: tune them with a kitchen thermometer for your coaster's chamber.
