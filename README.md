# LoRa Water Tank — Server / Client

Unifikovaný firmware a PCB pre **diaľkové dopĺňanie vodojemu** cez **EByte LoRa E220** (868 MHz). Jedna Arduino skica sa kompiluje ako **server** (čerpadlo / prietok / tlak) alebo **klient** (hladina vo vodojeme) podľa `#define DEVICE_ROLE`.

## Čo systém robí

- **Server** (pri zdroji vody / čerpadle): meria prietok a tlak, spína relé čerpadla, loguje na microSD, zobrazuje stav na OLED a komunikuje s klientom vo vodojeme.
- **Klient** (vodojem): meria výšku hladiny (QDY30A), číta min/max plaváky, posiela status a šetrí batériu (WOR sleep).
- **Logovanie a súbor so základným nastavením** - nastavenie naplnenia za deň.
- **Automatika**: štart pri nízkej hladine (min), stop pri plnom vodojeme (max), denný limit objemu, ochrana podľa tlaku.
- **Vzdialené príkazy** (UART / tretia strana cez LoRa): `start`, `stop`, `fill <m3>` (načerpať daný objem v m³).
- **Binárny protokol** s CRC16, rámce do ~64 B (nastavenie E220 `SPS_064`).

### Výber role pred flashom

V `Lora_U8G2_Menu_7_4.ino`:

```cpp
#define DEVICE_ROLE ROLE_SERVER   // alebo ROLE_CLIENT
```

LoRa adresy (fixed transmission): server `0x0001`, klient `0x0002`, externý riadič `0x0010`, channel `16`.

## Hardvér podľa schémy (EasyEDA)

Schéma a PCB: [`hardware/easyeda/`](hardware/easyeda/)  
Gerbery na výrobu: [`hardware/gerber/`](hardware/gerber/)

| Blok | Súčiastka / modul (zo schémy) | Úloha |
|------|-------------------------------|--------|
| MCU | **STM32F411CEU6 Black Pill V3.0** | riadenie, UART LoRa, I²C, SPI SD |
| Rádio | **LoRa E220 / E32 modul** (`LORAE32…`) | RF 868 MHz komunikácia |
| Displej | OLED 128×64 (U8G2 SSD1309/podobný) cez I²C |TZT2.4" - nutné upraviť rezistory I2C |
| Batéria / napájanie | meranie cez **INA219** | napätie batérie (server aj klient) |
| DC/DC | **LM2596S-5.0**, **AMS1117-3.3**, **LM1085S-3.3** | 5 V / 3,3 V regulácia |
| Relé | **G5LE-1-ASI DC3** + **2N2222A**, **PC817** | spínanie čerpadla (server) |
| Úložisko | **microSD** adaptér (SPI) | `log.txt`, `settings.txt` |
| RTC záloha | **CR2032** | záloha RTC |
| Ochrana | PTC `RXEF300`, diódy SK1010 / SS34 / 1N4004 | vstup / transienty |

### Použité senzory (externé)

| Senzor | Typ | Pripojenie / poznámka |
|--------|-----|------------------------|
| **QDY30A** | výška hladiny (4–20 mA / RS485 Modbus podľa zapojenia) | klient — UART levelmeter, softvér počíta % z cm |
| **YF-DN50** | prietokomer **10–200 L/min**, napájanie **5–24 V** - pravdepodobne bude nutný rezistor medzi 5V a IN | server — impulzy na pin **FLOW (PB0)**; softvér: ~12000 impulzov ≈ 1 m³ |
| **Vertikálny plavákový spínač** (plastový) | min / max hladina | klient — **TANK_MIN (PB14)**, **TANK_MAX (PB13)**, `INPUT_PULLUP`, aktívne LOW |
| **Senzor tlaku G1/4 0–10 MPa** | tlak v potrubí | server — analóg **PRESS (PA1)**; ochrana pri nízkom tlaku |


Ďalšie vstupy zo softvéru: **OCHRANA (PB13)** na serveri (ochranný kontakt), **IGNORE / tlačidlo MANUAL (PA8)**, **RELAY (PA9)**.

> Poznámka: na schéme sú siete / konektory pre FLOW, PRESS, RELAY, MAX, MIN a napájanie; názvy senzorov (QDY30A, YF-DN50, …) sú dokumentované tu podľa reálneho zapojenia, nie ako LCSC part number v EasyEDA knižnici.

## Softvér

- Platforma: **STM32duino** (odporúčané otestované s core 2.x; core 3.x používa `Uart` namiesto `HardwareSerial`)
- Kľúčové knižnice: `LoRa_E220` (EByte), U8g2, STM32RTC, INA219B, SdFat, IWatchdog
- Protokol: [`include/protocol.h`](include/protocol.h)
- Host sandbox (bez HW): [`sandbox/`](sandbox/) — `python sandbox/run_sandbox.py`

### UART menu (debug 115200)

`help`, `time`, `settime YYYY-MM-DD HH:MM:SS`, `status`, na serveri aj `start` / `stop` / `fill <m3>` / `synctime`.

## Štruktúra repozitára

```
Lora_U8G2_Menu_7_4_k_pia.ino   # unifikovaný firmware
include/                       # protocol.h, states.h
hardware/
  easyeda/                     # schéma + PCB JSON (EasyEDA)
  gerber/                      # ZIP + rozbalené Gerber súbory
sandbox/                       # Python testy protokolu
```

## Licencia / použitie

Projekt pre vlastnú automatizáciu vodojemu. PCB/Gerber sú exporty z EasyEDA; pred výrobou skontrolujte DRC a zapojenie napájania senzorov (5–24 V prietokomer, rozsah tlakového senzora).
