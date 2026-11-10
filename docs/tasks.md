# Задачі продукту (Lab 3 — super loop)

| ID | FR | Задача в циклі | Тип | Період / подія | Дедлайн (PRD) | Примітка |
|---|---|---|---|---|---|---|
| T1 | FR-01 | `task_imu` | періодична | 10 мс (100 Гц) | ≥100 Гц опитування | ±8g, \|a\| |
| T2 | FR-02 | `task_imu` | періодична | 10 мс | нахил >60° → інцидент | комплементарний фільтр |
| T3 | FR-03 | `task_baro` | періодична | 1000 мс | 1 Гц | T + P |
| T4 | FR-04 | `task_camera` | подія | інцидент | ≤300 мс до старту capture | GPIO QQVGA gray |
| T5 | FR-05/06/10 | `task_journal` | FSM | після кадру | цілісність + CRC | erase async, page/step |
| T6 | FR-07 | `arm` / інцидент | команда/подія | — | серво 180° | TIM3 PWM |
| T7 | FR-08 | `task_cli` | подія | UART RX | — | USART2 115200 |
| T8 | FR-09 | `task_led` | періодична | 500 мс / 50 мс | статуси | LD2 |
| T9 | — | `task_button` | подія | B1 debounce 40 мс | симуляція інциденту | latency → STATS |

## Головний цикл
`cli → button → imu → baro → camera → journal → led` + DWT stats / LOAD.
