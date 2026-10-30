# Інтеграція VIO → ROS 2/MAVROS → ArduPilot

Реалізовано автономні Python-інструменти та ROS 2-адаптер. **Реальне передавання
через MAVROS, приймання covariance ArduPilot і реакцію EKF не перевірено.**
SITL, ROS 2, Docker та системні залежності під час цієї роботи не запускалися
і не встановлювалися. Mock-тести не є доказом інтеграції.

## Наявні інтерфейси та архітектура

У `vio/vio_ekf.py` є пряме надсилання MAVLink у NED/FRD через порт 14551:
`VISION_POSITION_ESTIMATE` (21 елемент верхнього трикутника pose covariance),
`VISION_SPEED_ESTIMATE` (9 елементів speed covariance). ROS publisher відсутній.
VIO CSV має `t,x,y,z,vx,vy,vz,true_n,true_e,err_xy,quality,accepted,rejected`:
`t` — секунди від старту процесу, `x,y,z` — NED; covariance у CSV не записується.
За відсутності відео використовується імітований візуальний сенсор.

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

Docker networking **не налаштована цими файлами**. `127.0.0.1` усередині
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
| enu | east,north,up | метри, спільний origin/yaw |
| ned | x,y,z | метри, NED зі спільним origin |
| gps | lat,lon,alt | градуси WGS84 та метри, спільний datum висоти |

Час за замовчуванням у `timestamp`. Окремо для кожного файлу доступні
`--truth-time-column`/`--vio-time-column` і `--*-time-unit s|ms|us|ns|iso`.
ISO потребує timezone. MAVLink scaled GPS integers треба попередньо перевести
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

## Експеримент із якістю VIO — ще не виконаний

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
очікує ExternalNav velocity; новий speed-шлях реалізовано, але його приймання
та fusion ще треба перевірити на реальному стенді.
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
Результат локального запуску: **18/18 тестів пройшли**; CLI `--help`,
Python AST, XML parsing і `git diff --check` також пройшли.
Ці тести працюють без ROS/SITL/pymavlink. Реальні ROS message type support,
QoS discovery, launch, мережа, MAVLink covariance і EKF потребують стенду.

`.baseline.json` — SHA-256 знімок файлів поза `integration/` і початкового
`integration/Dockerfile.test` для перевірки незмінності в межах цієї роботи;
це не телеметрія й не результат реального експерименту.


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
| GPS1_TYPE | 0 | GPS1 disabled |

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

На стенді ще потрібно:

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
