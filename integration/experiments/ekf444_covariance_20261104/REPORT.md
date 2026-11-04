# Живий EKF3 covariance experiment — ArduCopter 4.4.4

**Підтверджено реакцію pose covariance та dropout/recovery у disarmed synthetic
сценарії. Speed covariance у цій версії ігнорується. Це не тест камерного VIO,
не політ і не підтвердження навігаційної точності.**

Фази: **2026-11-04 02:14:27.991–02:15:39.994 Europe/Kiev**
(00:14:27.991–00:15:39.994 UTC). Firmware
`9279df6191a772c098b2e4797c95ac954f258d88`, ARM64, образ за digest
`f5dffdb9f79ed8a7ff58aa7e6f6dfcbe2ebd2cead9fe7b20daea8d953b491540`.
GPS_TYPE=0; ExternalNav XY position та XYZ velocity; висота з барометра,
yaw з компаса. LOG_DISARMED=1, LOG_BITMASK=65535, file backend=1.
Точні значення до/під час досліду: [parameters.json](parameters.json).

## Сценарій й докази fusion

SITL залишався нерухомим, disarmed: усі 88 heartbeat у сирому capture мали
armed=false. SIMSTATE використовувався **тільки як synthetic simulator reference**
для lat/lon та attitude. Висота/вертикальна швидкість — явна модель Z=Vz=0
нерухомого апарата на землі; камера й IMU VIO не використовувалися.
До N/E позиції додано drift `(0.4*sin(ωt), 0.2*cos(ωt))` m, `ω=2π/5`;
швидкість отримує аналітичну похідну цього самого drift. Це контрольована
похибка синтетичного сенсора, а не реальний рух апарата. Сигнал повторюється
кожні 5 s, тому 20-секундні normal та inflated мають той самий заданий drift.

Задано EKF origin (-35.363261,149.165230,584 m), проведено 15 s warmup
(279 переданих пар; перша секунда без свіжого heartbeat не пересилалася).
Обидва cores повідомили **`is using external nav data`**, початкові position
і velocity; журнал містить `initialised`, `AHRS: EKF3 active`, origin set.
У DataFlash є VISP/VISV із точними remote timestamps переданих вимірів,
XKF3 innovations, XKF4 status/timeouts, XKV1 state covariance. Основна таблиця
стосується **core 0**, який був primary протягом усіх фаз. Initialized-біт
SS був встановлений у всіх використаних XKF4 записах; faults FS=0.

## Результати

| Фаза | s | Передано пар / VISP / VISV | VISP.PErr / VISV.VErr | XY position innovation RMS, m | XY velocity innovation RMS, m/s | Медіана P_NN, m² | Pos/vel timeout: записи XKF4 |
|---|---:|---:|---:|---:|---:|---:|---:|
| normal | 20 | 400 / 400 / 400 | 0.200 / 0.100 | 0.0400 | 0.2140 | 0.000985 | 0 / 487 |
| inflated | 20 | 400 / 400 / 400 | 3.63424 / 0.100 | 0.0305 | 0.2134 | 0.005784 | 0 / 487 |
| dropout | 12 | 0 / 0 / 0 | — | — | — | 0.808670 | 119 / 287 |
| recovery | 20 | 400 / 400 / 400 | 0.200 / 0.100 | 0.0119 | 0.2099 | 0.000995 | 0 / 487 |

Timeout-стовпчик: кількість sampled XKF4 записів, де **обидва** position та
velocity timeout bits були встановлені, / загальна кількість записів.
Це не кількість відхилених вимірів. Під час normal/inflated/recovery
абсолютна позиція була valid у всіх 487 записах кожної фази; у dropout —
168/287. Через **~6.998 s dropout** XKF4.TS змінився 48→51, тобто до
незмінних airspeed/drag timeout bits додалися pos/vel timeout. Перший XKF4
у recovery (~0.035 s після межі) вже мав TS=48. Точний момент скидання між
sampled записами невідомий. Після завершення всіх фаз input зупинився;
подальші `stopped aiding` у BIN належать цій остаточній зупинці input,
а не recovery чи середині 12-секундного dropout.

**Прийняті/відхилені виміри:** усі 1200 пар основного експерименту мають
відповідні VISP/VISV із Ign=0. Але в цьому backend Ign передається як false;
це не лічильник EKF innovation rejection. Точні EKF accepted/rejected counts
**недоступні** без додаткової firmware instrumentation. У sampled XKF4
активних фазах не було SP>1 або SV>1; це тільки вибірки gate-показників,
а не доказ, що всі 400 вимірів кожної фази пройшли gate. У dropout innovations
можуть бути збереженими старими значеннями — їхній RMS навмисно не порівнюється.

## Як саме використовується covariance у 4.4.4

Збережено код **саме образу цього запуску**, а не нового upstream:
[source_evidence.txt](source_evidence.txt),
[fusion_logic_source.txt](fusion_logic_source.txt).

- Pose: `posErr = cbrt(C[0]²+C[6]²+C[11]²)`, далі VISO noise floor.
  За діагоналей 0.01 / 4.0 результат після floor 0.2 — **0.2 / 3.63424**,
  що безпосередньо видно у VISP.PErr. EKF3 ExternalNav position гілка задає
  `R_OBS[3] = square(clamp(posErr,0.01,10))`. Отже змінюється шум виміру,
  використаний цією гілкою; це не повна 6×6 covariance fusion і не правильне
  стандартне sqrt(trace(C)) перетворення. Медіана P_NN зросла **у 5.87 раза**
  та повернулася близько baseline після recovery.
- Speed: handler взагалі не читає передану 9-елементну covariance.
  Backend передає `VISO_VEL_M_NSE=0.1`, що підтверджено незмінним VISV.VErr.
  Цей запуск **не може підтвердити реакцію EKF на speed covariance**.
- SP/SV та EKF_STATUS_REPORT — innovation consistency metrics, не передана
  VIO covariance і не P_NN. P_NN взято окремо з XKV1.V07, P_EE з V08.
- У старому firmware SS bit `using_gps` виставляється також за generic
  AID_ABSOLUTE/свіжою position pass, без перевірки source GPS. Цей біт не є
  доказом увімкненого GPS. Код, GPS_TYPE=0 і ExternalNav source перевірені;
  не інтерпретуйте назву цього біта ізольовано.

Є доказ обробки pose covariance, фактичної ExternalNav aiding, зміни
внутрішньої невизначеності EKF та реакції на втрату/повернення input.
Один послідовний disarmed дослід не визначає загальну якість weighting,
політний failsafe чи точність незалежної VIO. Менші normalized innovations
у inflated не означають кращого сенсора. Для точних rejection counts потрібна
instrumentation; для рухомої навігації — незалежні синхронні дані й окремий
контрольований експеримент. Польотних команд не виконано.

## Точні шляхи та відтворення аналізу

Шляхи від кореня репозиторію:

- `integration/experiments/ekf444_covariance_20261104/generator.csv` — фази,
  acquisition timestamps, усі передані covariance й send counts.
- `integration/experiments/ekf444_covariance_20261104/monitor.csv` — MAVLink
  CSV: 109 heartbeat, 1077 EKF_STATUS_REPORT, 1009 LOCAL_POSITION_NED.
- `integration/experiments/ekf444_covariance_20261104/received.jsonl` — сирі
  MAVLink дані, включно з SIMSTATE, heartbeat й STATUSTEXT.
- `integration/experiments/ekf444_covariance_20261104/synthetic_measurements.csv`
  — simulator reference, injected drift і synthetic measurements.
- `integration/experiments/ekf444_covariance_20261104/warmup.csv`.
- `integration/experiments/ekf444_covariance_20261104/dataflash_selected.csv`
  та `summary.json` — декодовані докази й метрики; `docker.log` — журнал SITL.
- **BIN:** `integration/sitl_runs/20261104T001121Z-7/logs/00000001.BIN`
  (22 298 624 bytes), SHA-256
  `09e8269e79d44ff20e2412c91f27cac0252d2433e08026b094d3cf273d87b25e`.
- **TLOG:** `integration/sitl_runs/20261104T001121Z-7/mav.tlog` та `.raw`.

BIN→Unix alignment побудований за 1200 matched VISP.RTimeUS/TimeUS;
розмах offsets ~28.3 ms, з EKF phase metrics виключено 0.25 s біля меж.
VISP/VISV counts зіставлені за точним measurement timestamp без цього guard.
Це sampled metrics, а не одна innovation на кожен send.

```bash
.venv/bin/python -B integration/analyze_live_ekf.py \
  integration/experiments/ekf444_covariance_20261104 \
  integration/sitl_runs/20261104T001121Z-7/logs/00000001.BIN
.venv/bin/python -B -m unittest discover -s integration/tests -v
```

39 офлайн-тестів пройшли, включно з періодичністю/похідною synthetic drift.
Для нового живого запуску користуйтеся наявним SITL launcher і задайте
**нову** директорію `--output` у `run_live_ekf.py`, паралельно запустивши
наявний `check_integration.py` на 14552. Після експерименту виконайте compose
stop і збережіть BIN саме нового timestamp-каталогу. Runner повторно не
використовує файли warmup/generator: відкриття в режимі `x` захищає результати.

**SITL зупинений (`Exited 143`); образи, контейнери та попередні результати
не видалялися. Робочі SITL/генератор/монітор і чужі компоненти не змінено.**
