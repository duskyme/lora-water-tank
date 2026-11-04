# LoRa vodojem sandbox

Hostiteľská simulácia bez hardvéru — overuje binárny protokol a hlavnú logiku server/klient/remote.

## Spustenie

```bash
cd sandbox
python run_sandbox.py
```

## Čo sa testuje

- framing `0xED…0xEF`, escape, CRC16
- veľkosť všetkých správ ≤ 64 B
- AUTO start pri min / stop pri max
- remote `FILL` v m³, `START`/`STOP`
- bezpečnosť (max, nízky tlak) počas remote
- potlačenie AUTO pri odpojenom klientovi
- sensor fault → plaváky (bit2)

Súbory: `protocol.py`, `sim.py`, `run_sandbox.py`
