# Базова лінія для Lab 4 (FreeRTOS)

## Сценарії (без змін методики)
Повторити `docs/scenarios.md`: **S1–S4** і L1–L5.
Команди: `STATS`, `MEM`, формат `EVT` / `LOAD` / `STAT`.
Збірка Lab 3: `git=1f0dbc4`.

## Метрики (Lab 3 — виміряно)
| Метрика | S1 | S2 | S3 | S4 | Вимога PRD | Спосіб | Вердикт ЛР3 |
|---|---|---|---|---|---|---|---|
| IMU exec avg (мкс) | 1566 | 1571 | 1566 | — | ≥100 Гц | DWT | OK у спокої |
| IMU jitter max (мкс) | 10571 | 417546 | **551472** | (≈capture) | малий | DWT/EVT | FAIL під capture |
| IMU missed | 25 | 21 | 27 | — | 0 бажано | STATS | FAIL під навант. |
| btn_latency / btn→inc max | — | 7.3 мс | 0¹ | **10.0 мс** | — | DWT/EVT | OK поза CAM |
| btn→capture_start max | — | 24.0 мс | — | **26.8 мс** | FR-04 ≤300 мс | EVT | **PASS** |
| camera max | — | 377 мс | 505 мс | **531 мс** | — | DWT/EVT | блок циклу |
| flash wall / OK rate | — | 414 мс / 1 | 399 мс / 1 | ~400 мс / **5/8** | FR-05/06/10 | EVT | FAIL rate у S4 |
| loop max | 18.6 мс | 425 мс | **552 мс** | — | — | DWT | ≈ camera |
| LOAD pct тип/пік | 17/17 | 17/45 | 17/54 | 17/**63** | — | DWT | |
| stack used (B) | 656 | 864 | 864 | — | запас | painting | OK |

¹ S3: `sim incident` без `btn_press`.

## Dual-measure
| Метрика | DWT | EVT | Δ |
|---|---|---|---|
| camera S2 | 377412 мкс | 390330 мкс | ~13 мс |
| camera S3 | 505073 мкс | 518166 мкс | ~13 мс |
| btn→incident S2 | 7344 мкс | 7345 мкс | ~0 |

LA не використовувався; другий канал — EVT з міткою МК + `analyze_log.py`.

## Гіпотези ЛР4
1. IMU jitter під S3/S4 ↓ на порядок.
2. Кнопка під час capture дає вимірювану latency ≪ 300 мс.
3. Flash FAIL rate при серійних інцидентах → ~0.
4. IMU missed під час journal → 0.
5. Додавання задачі без правки спільного циклу.

## Інструментарій
- `Platform/Src/perf.c`, CLI `STATS`/`MEM`
- `tools/analyze_log.py`, `tools/dump_frame.py`
