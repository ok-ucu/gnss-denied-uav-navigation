# Відтворюваний ArduPilot SITL на Apple Silicon

[Integration README](README.md) ·
[Живий covariance-звіт](experiments/ekf444_covariance_20261104/REPORT.md) ·
[Генератор і обмеження 4.4.4](test_ekf_covariance.md) ·
[Пакет для staging](STAGING.md).

Перевірено живий запуск на macOS M4 / Docker `linux/aarch64`.
Використовується готовий ARM64 образ із фіксованим digest; локальної збірки
ArduPilot немає. Підтверджено ArduCopter **4.4.4**, commit
`9279df6191a772c098b2e4797c95ac954f258d88`, image `linux/arm64`,
ELF arducopter `e_machine=183` (AArch64). Це перевірена, але стара версія:
висновки про її covariance не слід переносити на нові firmware без повторної
перевірки. Образ сторонній, не офіційний релізний образ ArduPilot.

Обраний [готовий образ](https://github.com/uxduck/ardupilot-sitl-docker)
містить compiled Copter і MAVProxy; tag `latest` не використовується для
повторного запуску. Зафіксовано manifest:

```text
orthuk/ardupilot-sitl@sha256:f5dffdb9f79ed8a7ff58aa7e6f6dfcbe2ebd2cead9fe7b20daea8d953b491540
```

## Основа Людини 1

Прочитано файли без checkout/merge:

```bash
git show origin/sitl-stand:sitl-stand/Dockerfile
git show origin/sitl-stand:sitl-stand/run.sh
git show origin/sitl-stand:sitl-stand/ext_nav.parm
```

Використаний ref `98a84ba861a5c9748517cc9437bc64f6a1c2cb33`.
Оригінальний Dockerfile клонує та збирає ArduPilot; run.sh використовує
host networking і pkill. Тут вони не виконуються й не змінюються.
`integration/sitl_ext_nav.parm` — точна копія її ExternalNav параметрів:
XY position і XYZ velocity — ExternalNav; height — barometer; yaw — compass.

У 4.4.4 нове ім'я `GPS1_TYPE` ще не працює: перший запуск виявив увімкнений
GPS. Тому окремий `sitl_transport.parm` додає сумісне `GPS_TYPE 0`,
`SERIAL0_PROTOCOL 2`, `AHRS_EKF_TYPE 3`, `EK3_ENABLE 1`. Саму копію параметрів
не змінено. Реальні параметри після повторного старту прочитані через MAVLink
на обох UDP-каналах і збігаються з очікуваними. GPS_RAW_INT пакети можуть
публікуватися навіть із вимкненим GPS; визначальною є конфігурація і fix,
а не сама наявність назви повідомлення.

## Підготовка й запуск

Усі shell-команди нижче — з кореня `gnss-denied-uav-navigation`.

```bash
git branch --show-current                    # має бути mavros-integration
docker info --format '{{.OSType}}/{{.Architecture}}'
docker pull --platform linux/arm64 orthuk/ardupilot-sitl@sha256:f5dffdb9f79ed8a7ff58aa7e6f6dfcbe2ebd2cead9fe7b20daea8d953b491540
docker image inspect orthuk/ardupilot-sitl@sha256:f5dffdb9f79ed8a7ff58aa7e6f6dfcbe2ebd2cead9fe7b20daea8d953b491540 --format '{{.Os}}/{{.Architecture}}'
mkdir -p integration/sitl_runs
docker compose -f integration/sitl.compose.yaml config
docker compose -f integration/sitl.compose.yaml up -d --no-build --pull never
docker compose -f integration/sitl.compose.yaml logs --follow
```

Ctrl+C для `logs --follow` завершує лише перегляд, не SITL. Дочекайтеся
`ArduPilot Ready` та `AHRS: EKF3 active` (первинне калібрування займає час).
Новий запуск entrypoint створює новий підкаталог у `sitl_runs/`; попередні
EEPROM, tlog і DataFlash файли не стираються. Одночасно працює лише один
compose service цього проєкту. У старті немає arm/takeoff, flight mode
switch, завантаження місії чи автоматичного vision sender.

При вже створеному зупиненому контейнері можна використати:

```bash
docker compose -f integration/sitl.compose.yaml start
```

Старт напряму викликає prebuilt arducopter і MAVProxy без sim_vehicle/waf.
Процеси контролює `start_sitl.sh`: завершення одного зупиняє інший;
SIGTERM/stop закриває їх. Образ, build path та defaults існування перевіряються;
відсутність бінарника завершує запуск замість прихованої збірки.

## Мережа macOS і два клієнти

```text
ARM64 arducopter ←→ TCP 127.0.0.1:5760 ←→ MAVProxy (в контейнері)
                                          ├─ UDP host.docker.internal:14556 → generator на Mac
                                          └─ UDP host.docker.internal:14552 → monitor на Mac
```

Host networking не потрібний. Docker Desktop надає `host.docker.internal`;
MAVProxy створює окремі outbound UDP sockets, і відповіді клієнтів повертаються
через них до автопілота. Тому container port publishing тут не потрібний.
На Mac клієнти слухають `0.0.0.0` (а не лише loopback); firewall має дозволяти
Python UDP. Не запускайте одночасно probe і справжні клієнти на тих самих
портах. Також не запускайте іншого vision sender або MAVROS як конкурента
генератору на ExternalNav вході.

Порти й адреса перевизначаються через `COVARIANCE_PORT`, `MONITOR_PORT`,
`SITL_HOST` перед compose. Після зміни відповідно змініть клієнтські URL.
Потрібні різні порти. Для іншого runtime/OS досяжну адресу хоста треба
перевірити окремо; тут перевірена саме Docker Desktop мережа цього Mac.

## Heartbeat та read-only перевірка

Наявний `.venv/bin/python` має pymavlink 2.4.50; системні залежності не
встановлювалися. Якщо середовища немає, за потреби створіть власне всередині
`integration/` і встановіть `pymavlink==2.4.50`; у наступних командах замініть
шлях Python. Самі команди підготовки в цій роботі не виконувалися:

```bash
python3 -m venv integration/.venv-sitl
integration/.venv-sitl/bin/python -m pip install pymavlink==2.4.50
```

Окремо від генератора й монітора запустіть:

```bash
.venv/bin/python -B integration/verify_sitl.py --timeout 20 \
  --output integration/sitl_connectivity.json
```

Probe одночасно відкриває обидва канали, перевіряє ≥2 MAVLink 2 heartbeat,
ATTITUDE, EKF_STATUS_REPORT, параметри EKF3/GPS/serial і disarmed стан.
`passed` означає **зв'язок і параметри**, а не навігаційну готовність.
`local_position_available` записується окремо. Немає arm або зміни EKF
параметрів; надсилаються тільки запити телеметрії та читання параметрів.

## Генератор і монітор одночасно

Термінал 1:

```bash
.venv/bin/python -B integration/check_integration.py \
  --conn udpin:0.0.0.0:14552 --duration 8 \
  --output integration/sitl_monitor_smoke.csv
```

Термінал 2 — короткий **синтетичний наземний transport smoke test**:

```bash
.venv/bin/python -B integration/test_ekf_covariance.py \
  --conn udpin:0.0.0.0:14556 --source csv \
  --measurements integration/sitl_grounded_smoke.csv \
  --duration 1 --rate 10 --output integration/sitl_generator_smoke.csv
```

`sitl_grounded_smoke.csv` містить явні нулі NED/velocity на 10 s для
нерухомого disarmed сценарію. Це не справжній VIO, не ground truth і не
доказ коректної навігації. Не використовуйте цей fixture для рухомого апарата.
Його єдина мета тут — перевірити send/receive transport та чотири фази без
польоту. Ніякого автоматичного arm або takeoff немає.

Для змістовного covariance експерименту замініть fixture на незалежні
синхронізовані вимірювання SITL/VIO, із правильними origin/attitude,
NED позицією та world NED XYZ velocity; див. [опис генератора](test_ekf_covariance.md).
Потрібно окремо встановити/перевірити EKF origin, source set, приймання
ExternalNav, часові затримки й відповідність траєкторії. Цей launch не
створює origin за замовчуванням. Режим `--source telemetry` очікує наявний
LOCAL_POSITION_NED, тому не може bootstrap-нути систему з відсутнім
зовнішнім позиційним входом. Повертання EKF output у власний input не є
незалежним експериментом. Можлива поява local position після VISION input
сама по собі не доводить правильного weighting covariance.

## Зупинка без видалення

```bash
docker compose -f integration/sitl.compose.yaml stop
docker compose -f integration/sitl.compose.yaml ps -a
```

Ці команди зупиняють лише сервіс цього compose-проєкту. Не використовується
`down`, `rm`, image prune або pkill. Після виконаної перевірки SITL зупинений;
створені контейнери `gnss-covariance-sitl-sitl-1` і короткої перевірки образу
`gnss-covariance-image-check` залишені, образ та логи збережені.

## Фактично перевірено

- Docker daemon linux/aarch64, manifest/image linux/arm64, ELF AArch64.
- Prebuilt ArduCopter 4.4.4 запустився, EKF3 active, GPS_TYPE=0.
- Генератор і монітор одночасно отримують дані на macOS, MAVLink 2,
  порти 14556 та 14552; конфлікту портів немає.
- Початковий наземний transport smoke test: 30 pose + 30 speed send calls.
  Його локальні CSV/JSON не входять до фінального пакета доказів.
- Наступний **контрольований synthetic/disarmed експеримент** підтвердив
  ExternalNav aiding, зміну ефективної pose error 0.2 → 3.63424,
  збільшення медіани P_NN у 5.87 раза, pos/vel timeout приблизно через 7 s
  dropout та скидання timeout у recovery. У кожній активній фазі BIN містить
  400 VISP і 400 VISV; у dropout — 0. Це не точні EKF acceptance counters.
- Код і DataFlash 4.4.4 підтверджують: **speed covariance ігнорується**,
  VISV.VErr залишається VISO_VEL_M_NSE=0.1.
- 39/39 офлайн-тестів у `.venv` із pymavlink, включно з MAVLink 2 encode/decode;
  без pymavlink — 38 passed, 1 skipped. Bash syntax та compose config перевірені.

[Повний звіт](experiments/ekf444_covariance_20261104/REPORT.md) містить таблицю,
innovation/status аналіз і точні шляхи. Канонічні докази:

- [DataFlash BIN](sitl_runs/20261104T001121Z-7/logs/00000001.BIN),
  [MAVLink TLOG](sitl_runs/20261104T001121Z-7/mav.tlog);
- [Фази генератора](experiments/ekf444_covariance_20261104/generator.csv),
  [монітор](experiments/ekf444_covariance_20261104/monitor.csv),
  [параметри](experiments/ekf444_covariance_20261104/parameters.json);
- [SHA-256 manifest](experiments/ekf444_covariance_20261104/SHA256SUMS.json).

Новий запуск використає новий timestamp-каталог. Не перезаписуйте
канонічний набір; для нового `run_live_ekf.py` задайте новий `--output`.
Raw TLOG `.raw`, terrain/EEPROM і декодований `dataflash_selected.csv`
збережені локально, але виключені з staging як runtime/похідні файли.

**Не перевірено:** реальні ROS 2/MAVROS launch та transport, інтеграція
камерного VIO, політний failsafe й навігаційна точність. SIMSTATE reference
і штучний drift не є камерним VIO. Точні innovation accepted/rejected counts
4.4.4 без instrumentation недоступні. Підтверджений результат обмежений
synthetic disarmed сценарієм зі [звіту](experiments/ekf444_covariance_20261104/REPORT.md).
