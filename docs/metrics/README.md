# Логи прогонів Lab 3

| Файл | Зміст |
|---|---|
| `S1.log` | Idle / простій (п. 6.1) |
| `S2.log` | Armed + інцидент B1 + повторні натискання (L2) |
| `S3.log` | `sim incident` + spam CLI |
| `S4.log` | Стрес: 8 інцидентів, flash OK/FAIL (п. 6.2) |
| `session.log` | Ранній повний сеанс |
| `results.md` | Зведені таблиці |
| `main_session.sal` | Запис Saleae (опційно) |
| `frame0.*` | Dump кадру |

```bash
python3 tools/analyze_log.py docs/metrics/S4.log
```

Звіт: `docs/lab3-report.md`. Baseline: `docs/baseline-lab4.md`.
