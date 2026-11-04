# Інтеграція VIO → ROS 2/MAVROS → ArduPilot

Модуль містить MAVLink-монітор, ROS 2/MAVROS-адаптер, аналізатори та
відтворюваний ARM64 SITL. Підтверджено окремі synthetic EKF3 та живі
Gazebo camera/motion smoke-тести. **Камерний VIO → EKF3 та політ не підтверджені.**

| Частина | Фактичний статус |
|---|---|
| Synthetic SITL normal → inflated → dropout → recovery | Live: ExternalNav aiding, pose covariance, innovations і timeout/recovery підтверджені DataFlash |
| Gazebo камера й окремий рух | Live ARM64: Image/Clock/IMU; рух через VelocityControl та знаки зсуву зображення при yaw=0 |
| SITL → motion bridge → Gazebo | BLOCKED: heartbeat/ATTITUDE є, LOCAL_POSITION_NED немає; лише cleanup zero Twist |
| ROS 2/MAVROS-адаптер | Офлайн-тести перетворень і covariance; реальні ROS messages, launch, discovery та MAVROS transport не перевірені |
| Камерний VIO → адаптер → ArduPilot | Не підключено й не перевірено; польоту та оцінки навігаційної точності не було |
| Speed covariance у ArduPilot 4.4.4 | Handler ігнорує передану матрицю; використовується VISO_VEL_M_NSE |

[Запуск і зупинка SITL](SITL.md) ·
[Генератор covariance](test_ekf_covariance.md) ·
[Повний живий звіт і докази](experiments/ekf444_covariance_20261104/REPORT.md) ·
[План фінального staging](STAGING.md).

У live-експерименті VISP.PErr змінився 0.2 → 3.63424, медіана P_NN core 0
зросла у 5.87 раза, pos/vel timeout з'явився приблизно через 7 s dropout,
а в recovery був скинутий. VISV.VErr залишився 0.1. Це підтверджує реакцію
цього firmware, а не точність VIO. Точні EKF accepted/rejected counts з
наявних логів не відновлюються; VISP/VISV Ign=0 не є innovation acceptance.

## Наявні інтерфейси та архітектура

У `vio/vio_ekf.py` є пряме надсилання MAVLink у NED/FRD через порт 14551:
`VISION_POSITION_ESTIMATE` (21 елемент верхнього трикутника pose covariance),
`VISION_SPEED_ESTIMATE` (9 елементів speed covariance). ROS publisher відсутній.
VIO CSV має `t,x,y,z,vx,vy,vz,true_n,true_e,err_xy,quality,accepted,rejected`:
`t` — секунди від старту процесу, `x,y,z` — NED; covariance у CSV не записується.
За відсутності відео використовується імітований візуальний сенсор.
У перевіреній гілці Людини 3 `origin/vio-setup` (`c5d31e2`) додано ROS 2
**вхід** sensor_msgs/Image, але не Odometry publisher. Її вихід — direct MAVLink
NED/FRD, а CSV також містить nis, n_inliers, sigma_v, latency_ms. Для нашого
адаптера потрібен окремо погоджений Odometry publisher/міст із правильними
ENU/FLU, twist covariance та ROS clock. Новий camera-режим VIO типово потребує
далекоміра (`--vertical range`). Базовий covariance-профіль не вмикає range;
новий opt-in `SITL_RANGE=1` вмикає simulated sonar. На землі він дає 0 см,
що нижче мінімуму та непридатне для масштабу камерного VIO. Використання
`--vertical ap` бере вертикаль із автопілота й має позначатися як залежний
loopback шлях. Код інших компонентів тут не змінюється.

`sitl-stand/Dockerfile` налаштовує лише UDP 14550/14551.
`sitl-stand/send_vision.py` повертає оцінку `LOCAL_POSITION_NED` у vision input;
це замкнений тест транспорту, а не незалежна VIO чи ground truth.
Поточні параметри стенду вибирають ExternalNav для горизонтальної позиції та
швидкості, барометр для висоти й компас для yaw; GPS вимкнений.

Новий шлях:

```text
майбутній VIO ROS publisher: nav_msgs/msg/Odometry (ENU/FLU)
  → integration/vio_adapter.py
  → /mavros/vision_pose/pose_cov: geometry_msgs/msg/PoseWithCovarianceStamped
  → MAVROS vision_pose plugin → VISION_POSITION_ESTIMATE → ArduPilot
  + /mavros/vision_speed/speed_twist_cov: geometry_msgs/msg/TwistWithCovarianceStamped
  → MAVROS vision_speed plugin → VISION_SPEED_ESTIMATE → ArduPilot
```

Це реальний [ROS 2 інтерфейс MAVROS](https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/vision_pose_estimate.cpp).
MAVROS перетворює ENU/FLU у NED/FRD і пакує covariance для MAVLink.
Адаптер також передає виміряну лінійну швидкість із covariance (див. нижче). Наявний VIO не підключений до нового шляху
автоматично: потрібен publisher, який реалізує описаний контракт.

## Контракт адаптера

- Вхід: `nav_msgs/msg/Odometry`, topic `/vio/odometry`, world `odom` (ENU),
  child `base_link` (FLU), pose саме корпусу, а не камери. Frame ID сам по собі
  не доводить напрям осей; відповідальність за них лежить на publisher.
- Позиція у метрах; нормалізований quaternion `(x,y,z,w)` задає body → world.
  ROS covariance — 6×6 row-major у порядку `(x,y,z,rotX,rotY,rotZ)`, одиниці
  m², rad² та m·rad для перехресних членів. Зберігаються всі 36 значень,
  включно з кореляціями. Невідома, all-zero, несиметрична, нескінченна або
  непозитивно-напіввизначена covariance відхиляється.
- Header timestamp зберігається без заміни часом отримання. Потрібен один
  ROS clock для джерела й адаптера; при симуляції — однаковий `use_sim_time`
  і `/clock`. Нульовий, повторний, зворотний, застарілий (>0.5 s) або
  майбутній (>0.05 s) час відхиляється. Після reset sim clock перезапустіть
  адаптер. За відсутності валідного VIO нові pose не публікуються.
- Вхідний QoS SensorData (best effort) сумісний із best effort/reliable
  publisher; вихід reliable depth 10 відповідає MAVROS subscriber.
- Адаптер не робить TF, вирівнювання yaw/origin чи extrinsics камери.
  Не можна просто перейменувати NED-кадр на `odom`. Для NED-позиції:
  `(E,N,U)=(y,x,-z)`; також окремо потрібні коректне перетворення quaternion
  FRD→FLU та covariance `C_out = J C_in Jᵀ` із повними перехресними блоками.
  `expand_upper_triangle()` лише відновлює симетричну 6×6 матрицю із 21 числа,
  **не** перетворює координати й не створює ROS-повідомлення із VIO автоматично.

Запуск у вже встановленому та активованому ROS 2 середовищі із MAVROS extras:

```bash
ros2 launch integration/mavros.launch.xml
python3 -B integration/vio_adapter.py --ros-args \
  -p input_topic:=/vio/odometry -p world_frame:=odom -p body_frame:=base_link
# При спільному симуляційному часі додайте: -p use_sim_time:=true
```

Можна перевизначити `output_topic`, `speed_topic`, `max_age`, `future_tolerance`.
Не запускайте одночасно прямий VIO MAVLink sender і MAVROS як два джерела vision.

## Порти й мережа

MAVROS за замовчуванням слухає окремий UDP **14555**, монітор — **14552**.
Для обох потрібні окремі виходи MAVProxy, наприклад у його консолі:

```text
output add HOST_REACHABLE_FROM_SITL:14555
output add HOST_REACHABLE_FROM_SITL:14552
```

Підставте реальну адресу хоста отримувача; ці рядки не є готовими shell-командами.
Перевизначення MAVROS URL:

```bash
ros2 launch integration/mavros.launch.xml fcu_url:='udp://0.0.0.0:14565@'
```

Launch включає штатний [MAVROS node.launch](https://github.com/mavlink/mavros/blob/ros2/mavros/launch/node.launch)
з upstream `apm_config.yaml`, локальним `mavros_plugins.yaml` і MAVLink 2
(потрібен для covariance extension). Запускайте з кореня репозиторію або
задавайте `pluginlists_yaml:=/absolute/path/to/integration/mavros_plugins.yaml`. Перевірте встановлену версію
MAVROS, активний plugin `vision_pose` та `ros2 topic info -v /mavros/vision_pose/pose_cov`.
Namespace можна перевизначити; тоді змініть і output_topic адаптера.

MAVROS launch сам не налаштовує Docker networking. Окремий
[перевірений SITL compose](SITL.md) забезпечує на Docker Desktop виходи через
host.docker.internal:14556 для генератора та :14552 для монітора. Вихід
14555 для MAVROS до цього compose автоматично не додано. `127.0.0.1` усередині
контейнера зазвичай означає сам контейнер. На Linux із host networking,
bridge network і Docker Desktop доступні адреси відрізняються. Якщо приймач
на хості — потрібна доступна з контейнера адреса хоста; якщо приймач у
контейнері — потрібні відповідна маршрутизація та UDP port mapping.
Перевірте firewall, напрям пакетів і зворотний MAVLink-трафік.
Наявний `run.sh` сам по собі не забезпечує виходи 14552/14555.

## MAVLink-монітор

Потребує `pymavlink`, імпорт відкладений: `--help` працює без нього.
Залежності можна встановлювати лише у власному підготовленому середовищі;
ця робота їх не встановлює.

```bash
python3 -B integration/check_integration.py \
  --conn udpin:0.0.0.0:14552 --connect-timeout 15 --telemetry-timeout 5 \
  --duration 120 --output integration/run_telemetry.csv
```

Очікування heartbeat автопілота обмежене тайм-аутом; GCS heartbeat ігнорується.
Після підключення фільтруються system/component джерела, запитуються 5 Hz
`EKF_STATUS_REPORT` і `LOCAL_POSITION_NED`. Прийняття запитів частоти не
гарантовано: воно залежить від автопілота й доступної пропускної здатності.
CSV записується і flush-иться після кожного рядка, без накопичення у RAM.
Кожне повідомлення має час отримання Unix та monotonic elapsed, тип і source.
Позиція — NED, `time_boot_ms` залишається окремим часом автопілота.
Поля інших типів порожні: попередня позиція не підставляється як свіжа.
`STATUSTEXT` записується по пакетах, без реконструкції багатопакетних текстів.

Втрата **всіх вибраних** повідомлень дає `telemetry_lost`, повернення —
`telemetry_restored`. Heartbeat може надходити без EKF/позиції: для перевірки
окремого потоку аналізуйте інтервали рядків відповідного типу. При втраті
монітор продовжує слухати, при I/O помилці завершується з кодом 1; з'єднання
закривається, вже записані рядки зберігаються. Без `--duration` зупинка Ctrl+C.

## Аналіз GPS ground truth і VIO

```bash
python3 -B integration/analyze_trajectories.py integration/gps.csv integration/vio.csv \
  --truth-format gps --origin -35.363261 149.165230 584 \
  --truth-time-unit ms --vio-format ned --vio-time-column t \
  --vio-time-unit s --vio-time-offset 0 --synchronized --max-gap 0.2
```

Це приклад схеми: вхідні CSV треба підготувати із реальних синхронних записів.
Формати:

| Формат | Колонки позиції | Одиниці |
|---|---|---|
| enu | east,north,up або east_m,north_m,up_m | метри, спільний origin/yaw |
| ned | x,y,z | метри, NED зі спільним origin |
| gps | lat,lon,alt | градуси WGS84 та метри, спільний datum висоти |

Час за замовчуванням у `timestamp`. Окремо для кожного файлу доступні
`--truth-time-column`/`--vio-time-column` і `--*-time-unit s|ms|us|ns|iso`.
Для SITL ENU CSV з `t_s` явно задайте `--truth-time-column t_s`;
порядок координат завжди East, North, Up. Це не синхронізує годинники саме
по собі. ISO потребує timezone. MAVLink scaled GPS integers треба попередньо перевести
у градуси/метри. GPS → ECEF → локальна ENU використовує WGS84 і явний origin.
MSL та ellipsoid altitude не можна змішувати без корекції datum.

`--synchronized` — ваше явне підтвердження спільної часової шкали після
перевірки логів. Без нього є попередження та відмова від порівняння.
`--vio-time-offset` додає виміряний зсув у секундах до VIO часу; інструмент
не оцінює його сам. Відносний `t` із поточного VIO не синхронізований із GPS
автоматично. Відсутній overlap, повтори й reset часу є помилкою.

Позиції ground truth лінійно інтерполюються на час VIO лише всередині overlap
і при інтервалі між опорними точками ≤ `max-gap`. Екстраполяції немає;
пропущені точки рахуються й викликають попередження. Потрібні ≥2 збіги.
Вихід JSON: горизонтальна/просторова похибка кожної пари, обидва RMSE,
дрейф як зміна вектора похибки між першою й останньою парою, m/s та % від
довжини ground truth шляху між зіставленими точками (для нерухомої траєкторії
відсоток null). Розріджені дані можуть занижувати довжину шляху.
Вирівнювання за першою точкою чи автоматичного повороту немає: початкове
зміщення входить у RMSE, але постійне зміщення не створює дрейф.
Наявний `true_n/true_e` без висоти не придатний для повної 3D оцінки.
`LOCAL_POSITION_NED`/`GLOBAL_POSITION_INT` — оцінки автопілота, не незалежний GT.
GPS є шумним референсом; у SITL для істинної траєкторії потрібні simulator logs.

## Межі підтвердженого експерименту та подальший VIO-тест

[Живий synthetic експеримент](experiments/ekf444_covariance_20261104/REPORT.md)
виконано через direct MAVLink із SIMSTATE reference та контрольованою
похибкою вимірів. Це не end-to-end ROS 2/MAVROS або камерний VIO тест.
Наведений нижче план стосується майбутнього камерного/MAVROS експерименту.

Розділяйте три різні речі:

1. **Передана covariance VIO:** збережіть вхідний Odometry і вихід адаптера
   (`ros2 bag record /vio/odometry /mavros/vision_pose/pose_cov /mavros/vision_speed/speed_twist_cov` із destination
   усередині `integration/`), а також MAVLink 2 capture для перевірки 21
   значення після перетворень MAVROS. ROS bag доводить публікацію, не приймання FCU.
2. **Діагностика EKF:** `ekf_pos_horiz_variance` і `ekf_velocity_variance` у CSV
   — сирі поля `EKF_STATUS_REPORT`, **не** передана VIO covariance і не
   матриця стану P у m². Їхню точну інтерпретацію перевіряйте для версії
   ArduPilot; [вихід EKF3](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp)
   використовує innovation consistency metrics. Не робіть висновок про
   covariance weighting лише за зростанням цього поля.
3. **Поведінка автопілота:** DataFlash EKF/external-nav логи, режими, failsafe,
   innovations, прийняті/відхилені виміри й незалежна помилка траєкторії.

На однаковій траєкторії окремо перевірте baseline; збільшення covariance
за незмінних pose; збільшення фактичної похибки; повний dropout; відновлення.
Запишіть точні версії firmware/MAVROS, параметри EK3/VISO, origin, clock,
час кожної фази та всі логи. Масштабувати слід усю covariance матрицю
додатним коефіцієнтом, зберігаючи кореляції, а не лише довільні діагоналі.
Не підмінюйте dropout повторною публікацією старої pose з новим часом.

Перед цим власникам інших компонентів потрібно додати ROS Odometry publisher
із коректною covariance/clock/extrinsics, незалежний GT із висотою та часовими
мітками, виходи MAVProxy 14552/14555 і погодити джерела EKF. Поточний стенд
очікує ExternalNav velocity. Direct MAVLink velocity input підтверджено у
synthetic SITL; speed-шлях через ROS/MAVROS ще не перевірено. В ArduPilot
4.4.4 speed covariance ігнорується незалежно від того, хто надіслав її.
Ці зміни поза `integration/` тут не реалізовані. Також потрібні origin/home,
перевірка yaw alignment, transport rate і налаштувань fusion перед польотом.

## Локальні перевірки

```bash
python3 -B -m unittest discover -s integration/tests -v
python3 -B integration/check_integration.py --help
python3 -B integration/analyze_trajectories.py --help
```

`tests/test_offline.py` використовує fake ROS messages і mock MAVLink transport:
перевіряє копіювання covariance/stamp, валідацію даних, timestamp формати,
GPS→ENU, RMSE/дрейф, інтерполяцію, відмову від несинхронних даних,
source filtering, втрату/відновлення телеметрії, CSV та I/O помилку.
Результат у підготовленому `.venv` з pymavlink: **58/58 тестів пройшли**; CLI `--help`,
Python AST, XML parsing і `git diff --check` також пройшли.
ROS/SITL для тестів не потрібні. Без pymavlink результат — 38 passed та
1 явно skipped wire serialization test. Реальні ROS message type support,
QoS discovery, launch і MAVROS transport потребують окремого ROS-стенду;
direct MAVLink та EKF перевірені живим synthetic експериментом.

`.baseline.json` — локальний допоміжний знімок, виключений зі здачі.
Канонічний набір доказів визначає
[SHA256SUMS.json](experiments/ekf444_covariance_20261104/SHA256SUMS.json).
[STAGING.md](STAGING.md) відокремлює первинні логи від відтворюваних похідних
файлів; `.gitignore` не видаляє наявні результати.


## Повторна перевірка launch та вимог EKF3

Локально не знайдено `ros2`, `/opt/ros`, Python-модулів `rclpy`,
`ament_index_python`, `launch_ros`. Отже **runtime-сумісність launch не
підтверджена**. Зіставлено лише XML і аргументи з поточною upstream гілкою
`ros2`: `mavros/launch/node.launch`, `apm.launch`, `apm_config.yaml`.
Конкретна встановлена на стенді версія може відрізнятися.

Виявлено, що штатний
[APM denylist](https://github.com/mavlink/mavros/blob/ros2/mavros/launch/apm_pluginlists.yaml)
містить `vision_speed_estimate`. Тому локальний launch тепер включає
`node.launch` безпосередньо та передає власний denylist, де speed plugin
увімкнений. Інші заборони APM збережені. Це конфігурація за upstream кодом,
а не результат запуску MAVROS.

Прочитаний `sitl-stand/ext_nav.parm` задає джерела першого набору:

| Параметр | Значення | Очікувані дані |
|---|---|---|
| EK3_SRC1_POSXY | 6 | ExternalNav XY position |
| EK3_SRC1_VELXY | 6 | ExternalNav XY velocity |
| EK3_SRC1_POSZ | 1 | Barometer height |
| EK3_SRC1_VELZ | 6 | ExternalNav vertical velocity |
| EK3_SRC1_YAW | 1 | Compass yaw |
| VISO_TYPE | 1 | MAVLink visual odometry backend |
| GPS1_TYPE | 0 | Ім’я у конфігурації Людини 1; у pinned 4.4.4 використовується GPS_TYPE=0 з окремого sitl_transport.parm |

Висота pose може передаватися, але POSZ тут вибрано з барометра;
вертикальна швидкість усе одно потрібна із ExternalNav. Повідомлення yaw
саме по собі не змінює вибраний компас як джерело yaw.
[Вибір джерел EKF3](https://ardupilot.org/copter/docs/common-ekf-sources.html)
не гарантує fusion: перевірте завантажені параметри, активний source set,
стан EKF, origin та firmware на стенді.

### Швидкість та covariance

Вхідний twist повинен відповідати стандартному контракту
[nav_msgs/Odometry](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/Odometry.msg):
pose у `header.frame_id`, twist у `child_frame_id`. Адаптер повертає обидва
вектори twist із FLU body в ENU world за quaternion pose. Матриця 6×6
перетворюється як `J C Jᵀ`, де `J=diag(R,R)`, включно з linear/angular
перехресними блоками. Лінійна швидкість — m/s, її covariance — (m/s)².
Це детерміноване обертання: додаткова невизначеність орієнтації не
поширюється, бо спільної pose/twist covariance в Odometry немає.
Publisher має забезпечити придатну оцінку невизначеності для цього припущення.

Вихід `/mavros/vision_speed/speed_twist_cov` має тип
`geometry_msgs/msg/TwistWithCovarianceStamped`, ENU world frame та той самий
stamp, що й pose. У [реальному speed plugin](https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/vision_speed_estimate.cpp)
використовується лише верхній лівий 3×3 блок лінійної covariance; далі MAVROS
перетворює ENU→NED для `VISION_SPEED_ESTIMATE`. Angular частина не надсилається
автопілоту цим MAVLink повідомленням.

Швидкість не вираховується різницюванням pose й не підставляється з EKF.
Відсутній twist, нульова/невідома linear covariance, відсутня невизначеність
будь-якої XYZ-компоненти або нечислові значення відхиляють усю пару pose/speed.
Виміряна нульова швидкість із валідною covariance дозволена. Відсутність
вертикального виміру не маскується вигаданим нулем. Якщо frontend має лише XY,
інтегратор VIO має надати обґрунтовану оцінку XYZ; цей адаптер її не створює.
Обидва повідомлення валідуються перед першою публікацією. Публікації у два
ROS topics не є атомарною транзакцією доставки; це потрібно перевірити у логах.

Covariance перевіряється масштабованим pivoted PSD алгоритмом, який допускає
вироджені PSD матриці, зокрема з нульовим першим pivot. Негативна діагональ,
несиметричність, indefinite матриці, NaN/Inf і all-zero відхиляються.
Числова точність симетричності/PSD — `1e-12` від максимального абсолютного
елемента, а не фіксовані одиниці; дуже малі негативні власні значення нижче
цього порога можуть бути невідрізненні від похибки округлення. Це числова
перевірка матриці, а не оцінка правдивості заявленої VIO невизначеності.

### Автономні сценарії та обов'язкові перевірки стенду

`tests/test_adapter.py` додає перевірки масштабу covariance (1e-24…1e24),
вироджених PSD/некоректних матриць, повороту швидкості й кореляцій на 90°,
stamp/frame та відмови від відсутніх даних. Сценарій із fake publishers:
нормальна covariance → covariance ×10 за незмінних даних → відсутність VIO
і відхилення запізнілого пакета → свіжі дані зі звичайною covariance.
Він перевіряє перетворення й відсутність replay, але **не реакцію ArduPilot**.
Без вхідних callback адаптер нічого не публікує; watchdog/failsafe автопілота
ці unit-тести не моделюють.

Для ROS 2/MAVROS та камерного VIO на стенді ще потрібно:

1. У sourced ROS 2 перевірити `ros2 pkg prefix --share mavros`, наявність
   `launch/node.launch`, `launch/apm_config.yaml`, їхні аргументи та версію
   пакета. Виконати `ros2 launch integration/mavros.launch.xml --show-args`,
   потім запуск із досяжним UDP-виходом SITL на 14555.
2. У стартовому журналі перевірити завантаження `vision_pose_estimate` та
   `vision_speed_estimate`; через `ros2 topic info -v` перевірити обидва
   subscribers, реальні типи, QoS і namespace. XML parsing цього не доводить.
3. Підключити справжній VIO ROS publisher із measured twist/covariance,
   перевірити quaternion, ENU/FLU, frame origin, єдиний clock і timestamps.
   Переконатися, що MAVLink 2 capture містить обидва VISION повідомлення та
   правильні covariance після перетворень.
4. На ідентичних траєкторіях повторити normal/inflated/dropout/recovery,
   записати ROS bag, MAVLink capture та DataFlash. Перевірити ExternalNav
   acceptance/fusion, реальні innovation metrics, режими й failsafe.
   Передану covariance VIO аналізувати окремо від діагностики EKF.

Opt-in simulated sonar and strict MAVLink input checks are documented in
[RANGE_INPUTS.md](RANGE_INPUTS.md). This separate disarmed check does not validate
Gazebo camera/SITL motion correspondence or replace the covariance experiment.


## Нові Phase 1 smoke-інструменти

| Інструмент | Призначення / залежності |
|---|---|
| [probe_vio_inputs.py](probe_vio_inputs.py) | Реальні MAVLink heartbeat, V2, range, IMU, attitude, local position, ACK та arrival gaps; Python + pymavlink |
| [sitl_range.parm](sitl_range.parm) | Opt-in internal SITL sonar: orientation 25, 20–5000 см; не Gazebo range |
| [start_sitl.sh](start_sitl.sh), [sitl.compose.yaml](sitl.compose.yaml) | `SITL_RANGE=0`, `SITL_STREAMRATE=10` за замовчуванням; для per-message requests використовувати `SITL_STREAMRATE=-1` |
| [probe_camera.py](probe_camera.py) | Реальні ROS Image/Clock, dimensions/encoding, FPS, stamps, ненульові bytes; ROS 2 Jazzy, sensor_msgs, rosgraph_msgs |
| [probe_camera_motion.py](probe_camera_motion.py) | Короткий stationary → +X → pause → +Y → stop через наявний cmd_vel; PNG, LK, Gazebo odometry; додатково geometry_msgs, nav_msgs, NumPy, OpenCV |

Офлайн-тести не потребують ROS/SITL/Gazebo: ROS/OpenCV імпортуються лише під час
запуску відповідних probes. Для MAVLink wire/error tests потрібний pymavlink:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install pymavlink
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m unittest discover -s integration/tests -v
```

Команди виконувати з кореня репозиторію. [Запуск sonar і MAVLink probe](RANGE_INPUTS.md)
використовує UDP 14553; 14554 — окремий diagnostic output. Не прив'язувати два
listeners до одного порту. Default covariance/monitor залишаються 14556/14552,
VIO — 14551, MAVROS — 14555. ACK не є доказом фактичної доставки повідомлення.

### ARM64 Gazebo runtime без зміни коду Людини 2

Перевірений образ `gz-sim` має `linux/arm64`, ROS Jazzy, Gazebo Harmonic,
ros_gz_bridge, NumPy і OpenCV. Host macOS не потребує ROS або OpenCV. Навіть `--help` для motion probe
потребує його ROS/OpenCV runtime; виконувати його в контейнері, а не host `.venv`.
Якщо образ уже є, збірку пропустити:

```bash
docker image inspect gz-sim --format '{{.Os}}/{{.Architecture}}'
```

За потреби збірка з існуючого Dockerfile commit `efd7f41cda82052417f4d5caae34bb25f2700ab7`:

```bash
git show efd7f41cda82052417f4d5caae34bb25f2700ab7:gazebo/Dockerfile |
  docker build --platform linux/arm64 -t gz-sim -
```

У зафіксованому запуску HTTP Ubuntu index дав Hash Sum mismatch. Мінімальний
workaround лише для stdin build input (checksum verification залишається):

```bash
python3 - <<'PYBUILD' | docker build --platform linux/arm64 -t gz-sim -
import subprocess
source = subprocess.check_output([
    'git', 'show',
    'efd7f41cda82052417f4d5caae34bb25f2700ab7:gazebo/Dockerfile'], text=True)
print(source.replace('RUN apt-get update',
    "RUN sed -i 's|http://ports.ubuntu.com|https://ports.ubuntu.com|g' "
    '/etc/apt/sources.list.d/ubuntu.sources && apt-get update'))
PYBUILD
```

Це не змінює tracked Dockerfile. Mutable base tag та apt packages не гарантують
байт-ідентичної майбутньої збірки; виміряні image/source IDs наведені у звіті.

### Статична камера або рух без ArduPilot/VIO

Наступний самодостатній приклад не читає локальні архіви старих запусків. Він
витягує незмінений SDF із Git і використовує committed probes. Для руху замінити
`MODE=static` на `MODE=motion`; це керує лише віртуальною Gazebo-платформою.

```bash
MODE=static
RUN_DIR="$PWD/integration/camera_probe_runs/$(date -u +%Y%m%dT%H%M%SZ)_$MODE"
CONTAINER="gnss-camera-$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$RUN_DIR"
git show efd7f41cda82052417f4d5caae34bb25f2700ab7:gazebo/worlds/camera_world.sdf > "$RUN_DIR/camera_world.sdf"
(
trap 'docker stop --timeout 15 "$CONTAINER" >/dev/null 2>&1 || true' EXIT
docker run --name "$CONTAINER" --platform linux/arm64 --init -e MODE="$MODE" \
  --mount type=bind,src="$PWD/integration",dst=/integration,readonly \
  --mount type=bind,src="$RUN_DIR",dst=/results \
  gz-sim bash -lc '
    source /opt/ros/jazzy/setup.bash
    export LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=47 GZ_PARTITION=gnss_camera_probe
    gz sim -v4 -s -r --headless-rendering /results/camera_world.sdf > /results/gazebo.log 2>&1 &
    GZ_PID=$!
    ros2 run ros_gz_bridge parameter_bridge \
      "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" \
      "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
      "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU" \
      "/model/camera_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
      "/model/camera_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist" > /results/bridge.log 2>&1 &
    BRIDGE_PID=$!
    trap "kill $GZ_PID $BRIDGE_PID 2>/dev/null || true; wait" EXIT
    trap "exit 143" TERM
    trap "exit 130" INT
    if [ "$MODE" = static ]; then
      python3 /integration/probe_camera.py --warmup 10 --duration 10 --output /results/probe.json
    elif [ "$MODE" = motion ]; then
      python3 /integration/probe_camera_motion.py --output-dir /results
    else
      exit 2
    fi'
)
```

Зберігати результат, навіть якщо probe повернула FAIL. Контейнер завершується
після probe, EXIT trap зупиняє дочірні процеси; parent trap також виконує docker stop.
Не запускати ці приклади одночасно в тому самому ROS domain/Gazebo partition.
Вихідний mono PNG у motion probe — grayscale-представлення реального rgb8 кадру.
Файл `flow.csv` містить LK pixel displacement, а не валідовану навігаційну швидкість.

### Переносимі результати та межі доказів

- [ARM64 camera live test](reports/arm64_camera.md): PASS лише статичного сенсорного транспорту.
- [Gazebo motion test](reports/gazebo_motion.md): PASS руху й знаків осей при identity attitude; точність flow не підтверджена.
- [SITL range/input diagnosis](reports/sitl_range_inputs.md): transport PASS, range validity/local position FAIL.
- [Motion bridge smoke-test](reports/motion_bridge.md): BLOCKED, не end-to-end PASS.

Первинні дані залишаються локально в `camera_probe_runs/`, `input_probe_runs/`,
`motion_bridge_runs/`. Вони навмисно не є GitHub-посиланнями та не потрібні для
виконання наведених команд. Нові звіти — стислий переказ вимірювань; raw evidence
не додається повторно. Живі тести ROS 2/MAVROS adapter, camera VIO → EKF3, GNSS-denied
flight і capture-to-MAVLink latency ≤50 мс залишаються NOT TESTED.
Історичні [STAGING.md](STAGING.md) / [STAGING_FILES.txt](STAGING_FILES.txt) описують
попередній covariance commit, а не новий список staging.
