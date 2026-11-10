# Сценарії навантаження (Lab 3)

Одна й та сама збірка (`git=` у банері). Результати → `docs/metrics/`.

## Обов’язкові (п. 6.1)

### S1 — простій
1. Прошивка, дочекатись self-test.
2. `STATS clear`, почекати **30 с** без команд і без руху.
3. `STATS`, `MEM`.
4. Зберегти UART-лог як `docs/metrics/S1.log`.

### S2 — номінальний + події кнопки (також L2)
1. `arm`, рідкісні `status`, спокійне положення плати.
2. Один інцидент (B1), далі багаторазові натискання B1 у стані INCIDENT / навколо довгих операцій.
3. `STATS`, `MEM` → `docs/metrics/S2.log`.

### S3 — найгірший (UART spam)
1. `STATS clear`, `arm`.
2. `sim incident` (або B1) — довгий capture + flash FSM.
3. Одночасно спамити `status` / `help` по UART.
4. `STATS`, `MEM` → `docs/metrics/S3.log`.

### S4 — стрес меж (п. 6.2, агреговано)
1. Цикл: `arm` → B1 (інцидент) → дочекатись flash → `reset` → знову.
2. Повторити **≥5** повних інцидентів; мішати `status`/`help`.
3. Лог → `docs/metrics/S4.log` (бажано завершити `STATS`/`MEM`).

## Межі super loop (п. 6.2) — коротко
| # | Сценарій | Як відтворити | Статус у логах |
|---|---|---|---|
| L1 | Довга операція + IMU | S3/S4 vs S1 (`IMU_JITTER`) | ✅ S1↔S3/S4 |
| L2 | Подія під час / після capture | S2: 27× B1 | ✅ (див. results) |
| L3 | Потік команд | S3 spam CLI | ✅ |
| L4 | Відмова I²C | від’єднати MPU | ⏳ якісно в limitations |
| L5 | Точка зламу | S4: 8 інцидентів, 3× flash FAIL | ✅ частково |

## Команди вимірювання
```
STATS          # min/avg/max задач + IMU_JITTER
STATS clear
MEM            # stack usage (painting)
```
Автоматичні рядки: `EVT t_us=... name=...`, `LOAD pct=...`.

```bash
python3 tools/analyze_log.py docs/metrics/S4.log
```
