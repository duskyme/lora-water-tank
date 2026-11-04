# Hardvér — LoRa Water Server / Client PCB

## Súbory

| Súbor | Popis |
|-------|--------|
| `easyeda/SCH_Lora_Water_Server_2026-11-04.json` | EasyEDA schéma (2 sheets) |
| `easyeda/PCB_Lora_Water_Server_2026-11-04.json` | EasyEDA PCB |
| `gerber/Gerber_Lora_Water_Server.zip` | Gerber archív na výrobu |
| `gerber/extracted/` | Rozbalené vrstvy (GTL/GBL, masky, drill, outline) |

Import schémy/PCB: EasyEDA → File → Open / Import JSON.

## Bloky zo schémy (prehľad)

**Sheet 1 — napájanie**
- Vstup cez svorkovnicu, PTC poistka, usmerňovač / ochranné diódy
- **LM2596S-5.0** → 5 V
- **AMS1117-3.3** / **LM1085S-3.3** → 3,3 V pre MCU a logiku
- LED indikácia

**Sheet 2 — riadenie a I/O**
- **STM32F411 Black Pill**
- **LoRa E220/E32** modul
- **INA219** (I²C meranie batérie)
- **microSD** (SPI)
- **Relé G5LE** + optočleny **PC817** + **2N2222A**
- Konektory: FLOW, PRESS, RELAY, MAX/MIN plaváky, napájanie senzorov
- **CR2032** pre RTC

## Externé senzory (mimo PCB footprint názvov)

Pozri hlavný [README.md](../README.md): QDY30A, YF-DN50, plaváky, tlak G1/4 0–10 MPa.
