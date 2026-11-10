# Таблиці результатів Lab 3

Прошивка `git=1f0dbc4`. Сирі логи: `S1.log` … `S4.log`, `session.log`.

## STATS (DWT)

| name | S1 n / avg / max (мкс) | S2 n / avg / max | S3 n / avg / max |
|---|---|---|---|
| imu | 2505 / 1566 / 1569 | 3151 / 1571 / 18221 | 2229 / 1566 / 1569 |
| baro | 25 / 18562 / 18564 | 32 / 18358 / 18700 | 24 / 18449 / 18699 |
| camera | 0 | 1 / 377412 / 377412 | 1 / 505073 / 505073 |
| journal | 0 | 83 / 844 / 16069 | 83 / 844 / 16246 |
| loop | 4.27e6 / 4 / 18569 | 5.25e6 / 4 / 425255 | 3.70e6 / 5 / 552470 |
| btn_latency | 0 | 1 / 7344 / 7344 | 1 / 0 / 0 |
| IMU_JITTER max / missed | 10571 / 25 | 417546 / 21 | 551472 / 27 |

> У `S4.log` фінального блоку `STATS`/`MEM` немає (лог обірвано після reboot); метрики S4 — з EVT.

## EVT latencies (мкс)

| Інтервал | S2 | S3 | S4 (n=8 інцидентів) |
|---|---|---|---|
| btn_press → incident | 7345 | — | min 7171 / med 7523 / **max 9995** |
| incident → capture_start | 16656 | 17248 | min 16431 / med 17234 / max 17252 |
| capture_start → done | 390330 | 518166 | min 340174 / med 456451 / **max 531251** |
| flash_write_start → done | 414403 | 399185 | n=5: min 395710 / med 415495 / max 418383 |
| btn_press → capture_start | 24001 | — | min 24001 / med 24766 / **max 26811** |

## MEM / LOAD

| | S1 | S2 | S3 | S4 |
|---|---|---|---|---|
| STACK used / free | 656 / 368 | 864 / 160 | 864 / 160 | (не знято) |
| LOAD тип / пік | 17 / 17 | 17 / 45 | 17 / 54 | 17 / **63** |

## S2 — повторні натискання (межа L2)
- `btn_press` = **27**, `incident` = **1** (решта в стані INCIDENT ігноруються FSM).
- Натискань **під час** вікна `[CAM]` = **0** (події з’являються лише після кінця capture/flash — наслідок блокувального GPIO-захоплення).

## S4 — стрес / точка зламу (п. 6.2)
| Показник | Значення |
|---|---|
| Інцидентів (повний цикл) | **8** |
| Flash OK (`Journal slot … saved`) | **5** |
| Flash FAIL (`Header verify FAIL`) | **3** (~38 %) |
| FR-04 max `btn→capture_start` | **26.8 мс** (PASS ≤300 мс) |
| Capture max | **531 мс** |
| LOAD пік | **63 %** |
| Ребутів у логу | ≥1 (сесія містить повторний банер) |

## Dual-measure
| Метрика | DWT STAT | EVT | Δ |
|---|---|---|---|
| camera S2 | 377412 | 390330 | ≈13 мс |
| camera S3 | 505073 | 518166 | ≈13 мс |
| btn→incident S2 | 7344 | 7345 | ≈0 |

## Команди
```bash
python3 tools/analyze_log.py docs/metrics/S1.log
python3 tools/analyze_log.py docs/metrics/S2.log
python3 tools/analyze_log.py docs/metrics/S3.log
python3 tools/analyze_log.py docs/metrics/S4.log
```
