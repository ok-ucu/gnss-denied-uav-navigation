# Контрольований ExternalNav covariance experiment

`test_ekf_covariance.py` перевіряє генерацію та надсилання синтетичних вимірів.
Успішний send не означає доставку UDP, приймання ExternalNav або fusion EKF3.
Поточний результат: **живий synthetic/disarmed SITL 4.4.4 експеримент
виконано**, pose covariance та dropout/recovery підтверджені DataFlash.
Speed covariance цей firmware ігнорує. Це не камерний VIO, не політ і не
перевірка ROS 2/MAVROS transport.

[Integration README](README.md) · [Запуск SITL](SITL.md) ·
[Звіт і первинні докази](experiments/ekf444_covariance_20261104/REPORT.md).

На цьому Mac перевірено готовий ARM64-образ із pinned digest та наявний
`.venv/bin/python` із pymavlink 2.4.50. Системні залежності не встановлювалися.
Підготовку й зупинку середовища описано у [SITL.md](SITL.md).

## Запуск після підготовки стенду

Запускайте з кореня репозиторію у середовищі з pymavlink:

```bash
python3 -B integration/test_ekf_covariance.py \
  --conn udpin:0.0.0.0:14556 --heartbeat-timeout 15 \
  --duration 10 --dropout-duration 5 --recovery-duration 15 --rate 30 \
  --normal-variance 0.01 --inflated-variance 4 \
  --output integration/covariance_test.csv
```

14556 — окремий порт експерименту, можна перевизначити. Додайте окремий
MAVProxy output до доступної з SITL адреси отримувача на цьому порту.
Docker networking автоматично не налаштовується; localhost контейнера
не обов'язково є хостом. Не запускайте одночасно інший vision sender/MAVROS
на цьому самому ExternalNav вході. Інструмент не змінює параметри EKF,
не озброює апарат, не задає origin/home і не керує траєкторією SITL.

Фази завжди normal → inflated → dropout → recovery. `--duration` задає
типову тривалість, `--normal-duration`, `--inflated-duration`,
`--dropout-duration`, `--recovery-duration` — окремі перевизначення.
Частота всіх фаз задається `--rate`; у dropout є тільки CSV ticks, обидва
VISION повідомлення не надсилаються. Пропущені слоти не створюють send burst.
У inflated змінюється position/speed variance, attitude variance залишається
сталою (`--attitude-variance`, default 0.1 rad²), щоб ізолювати один фактор.
Усі параметри мають бути скінченними додатними, inflated > normal.

## Джерела вимірів і координати

Default `--source telemetry` запитує `LOCAL_POSITION_NED` та `ATTITUDE`
із заданою частотою. Пари повинні надходити від вибраного heartbeat
system/component, мати різницю boot timestamp ≤ `--max-skew` (0.05 s)
і вік отримання ≤ `--max-age` (0.5 s). Обидва VISION повідомлення отримують
один measurement timestamp: `LOCAL_POSITION_NED.time_boot_ms * 1000`.
Повторні/старі timestamps не пересилаються; reset/wrap boot clock завершує
експеримент. Час отримання не доводить відсутності мережевої затримки: великі
backlog/delay треба виключити на стенді. Фільтр використовує найближчу доступну
свіжу attitude, без інтерполяції; для швидких обертань зменшуйте max-skew.

Це **повернення оцінки EKF у власний вхід**, а не незалежний VIO/ground truth.
Режим корисний для діагностики транспорту, але не доводить коректну навігацію
чи коректне зважування вимірів. Фіксовану позицію скрипт більше не вигадує.

Для незалежних підготовлених вимірів доступний CSV replay:

```bash
python3 -B integration/test_ekf_covariance.py --source csv \
  --measurements integration/planned_measurements.csv --duration 10 --rate 30 \
  --output integration/covariance_replay.csv
```

Обов'язкова схема CSV:

```text
elapsed_s,north_m,east_m,down_m,roll_rad,pitch_rad,yaw_rad,vnorth_m_s,veast_m_s,vdown_m_s
```

`elapsed_s` — секунди від початку експерименту, не від старту SITL. Початок
встановлюється після heartbeat, перед normal; timestamps replay — Unix us
від цього початку + час рядка. Використовується останній доступний рядок
не пізніше поточного часу; старіший за max-age відхиляється. Немає
екстраполяції старої pose, зміни її timestamp на свіжий або повторного send
одного timestamp. CSV мусить покривати всі активні фази, включно з recovery.
Повна відсутність успішної пари у будь-якій активній фазі завершується помилкою.

Оператор має узгодити початок CSV із реальною траєкторією SITL, origin,
напрямками осей і швидкістю симуляції. Скрипт не запускає місію, не вирівнює
траєкторії й не перевіряє кінематичну відповідність рядків реальному апарату.
Довільний запис минулого польоту не стає синхронним ground truth від replay.
Для живого independent simulator/VIO джерела Python API `experiment()` також
приймає об'єкт `sample(elapsed_s, monotonic_now) -> Measurement | None`;
provider відповідає за acquisition timestamp, clock і свіжість вимірів.

В обох режимах позиція й лінійна швидкість — **world NED** у m та m/s;
attitude — body FRD → NED, roll/pitch/yaw у rad. CSV із ENU не можна подавати
без попереднього правильного перетворення позиції, швидкості та attitude.
Швидкість не диференціюється із pose і не підставляється нульовою.

## MAVLink 2 та CSV результату

Енкодер створюється явно з `pymavlink.dialects.v20.ardupilotmega`, без
залежності від порядку попередніх імпортів mavutil. Імпорт самого інструмента
не змінює MAVLINK20 середовища й не потребує pymavlink. Covariance:

- pose — 21 значення верхнього трикутника 6×6 `(x,y,z,roll,pitch,yaw)`;
  діагональні індекси 0,6,11,15,18,20;
- speed — 9 значень 3×3 row-major, діагональ 0,4,8;
- задаються **variance**, не standard deviation: m², rad², (m/s)²;
- reset_counter залишається 0: dropout сам по собі не означає reset VIO frame.

Результат CSV flush-иться після кожного рядка. Він містить phase_start/end,
phase elapsed, monotonic elapsed та Unix час, source, measurement timestamp,
передані position/attitude/velocity, configured variance, фактично передані
covariance як JSON, прапорці та лічильники send calls окремо для pose/speed,
у фазі та за весь запуск. У dropout covariance_sent порожня. Якщо pose send
успішний, а speed send падає, рядок `send_error` зберігає частковий результат.
Є записи no_fresh_measurement, duplicate_or_old_measurement, error/interrupted.
Лічильники означають завершені виклики pymavlink; транспорт може не повідомити
про втрату UDP. Часова шкала CSV результату не є шкалою EKF DataFlash.

Heartbeat очікується з deadline, heartbeat GCS ігнорується. При помилці код
завершення 1, при Ctrl+C — 130; з'єднання закривається у finally.
Відсутність heartbeat/помилки до створення CSV повідомляються через stderr.

## Що підтверджено саме для ArduPilot 4.4.4

Перевірено код образу commit `9279df6191a772c098b2e4797c95ac954f258d88`,
а не новий upstream master. Фрагменти коду та логи збережено у
[звітному наборі](experiments/ekf444_covariance_20261104/REPORT.md).

- Pose covariance: handler обчислює
  `posErr = cbrt(C[0]²+C[6]²+C[11]²)` та аналогічно attitude error;
  backend застосовує VISO noise floors. Це стара специфічна формула,
  не sqrt(trace(C)) і не повна матрична fusion. За діагоналей 0.01 та 4.0
  виміряний VISP.PErr після floor становить 0.2 та 3.63424.
- EKF3 ExternalNav position гілка використовує квадрат обмеженого posErr
  як observation noise. Медіана P_NN core 0 у inflated зросла у 5.87 раза,
  а у recovery повернулася близько baseline.
- **Speed covariance не читається handler цієї версії.** Незалежно від
  переданих дев'яти чисел backend використовує VISO_VEL_M_NSE=0.1;
  VISV.VErr у всіх активних фазах залишився 0.1. Змінювати speed covariance
  цим інструментом можна, але реакцію саме на неї 4.4.4 не демонструє.

Після 15 s warmup проведено normal 20 s → inflated 20 s → dropout 12 s →
recovery 20 s при 20 Hz. SIMSTATE — synthetic simulator reference;
додано повторюваний drift позиції та його аналітичну похідну для швидкості.
Обидва cores підтвердили ExternalNav aiding. BIN має по 400 VISP/VISV у
кожній активній фазі. При dropout input відсутній, приблизно через 7 s
з'являються pos/vel timeout, а у recovery вони скидаються.

VISP/VISV Ign=0 не означає, що кожен вимір пройшов EKF innovation gate:
у цьому backend це поле передається як false. Точні per-measurement
acceptance/rejection counters у доступних логах відсутні. XKF3 innovations,
XKF4 gate/status та XKV1 state covariance аналізуються окремо від кількості
send calls. EKF_STATUS_REPORT не є переданою covariance.

Для нового firmware повторно перевіряйте його код, VISO/EK3 параметри й
DataFlash: висновки 4.4.4 не можна переносити на іншу версію. Для камерного
VIO потрібні незалежні синхронні дані, підтверджений ROS/MAVROS шлях та
окремий baseline; польотні failsafe цим дослідом не перевірені.

## Офлайн-перевірки

```bash
python3 -B -m unittest discover -s integration/tests -v
python3 -B integration/test_ekf_covariance.py --help
```

Тести перевіряють layouts covariance, некоректні параметри, фази/dropout,
часткову помилку send, лічильники, timestamps, CSV replay, freshness/skew,
GCS filtering, heartbeat timeout і закриття transport через mocks.
Окремий тест робить справжнє MAVLink 2 encode/decode лише за наявності
pymavlink; інакше явно skipped. Він теж не запускає SITL.
Поточний результат: **39/39 passed** у `.venv` із pymavlink; без pymavlink —
38 passed, 1 skipped. Живий direct-MAVLink SITL результат наведено у
[REPORT.md](experiments/ekf444_covariance_20261104/REPORT.md); реальний
ROS 2/MAVROS та камерний VIO не перевірені. Фінальний набір доказів і правила
збереження результатів наведено у [STAGING.md](STAGING.md).
