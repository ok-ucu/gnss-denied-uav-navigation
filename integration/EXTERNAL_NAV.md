# ExternalNav output validation (offline)

`analyze_external_nav.py` перевіряє **спостережувані wire messages**
`VISION_POSITION_ESTIMATE` і `VISION_SPEED_ESTIMATE`, а не send-call CSV та не
кількість вимірювань, прийнятих EKF3. Потрібні Python 3 та `pymavlink`
(наявні integration dependencies); ROS, NumPy і SITL не потрібні.
Нові тести — синтетичні offline fixtures, **не нова live-перевірка VIO**.

```bash
# Із кореня репозиторію; stdout містить JSON, input відкривається read-only.
.venv/bin/python integration/analyze_external_nav.py vio_rx.tlog \
  --system 42 --component 191 --min-rate 20 --max-gap 0.2
# Raw concatenated MAVLink packets: зміст перевіряється, timing NOT TESTED.
.venv/bin/python integration/analyze_external_nav.py vio_wire.bin --raw
.venv/bin/python -B -m unittest discover -s integration/tests -v
```

System/component треба взяти з фактичного VIO sender, а не копіювати 42/191
з прикладу. Без фільтра змішування кількох sender одного типу дає FAIL.
Файли з нестандартним framing (наприклад JSONL чи PCAP) спочатку потребують
експорту стандартного timestamped `.tlog` або raw MAVLink bytes; розширення
файла саме по собі не визначає формат. Пошкоджені CRC/framing і обрізаний
пакет дають FAIL, без тихого пропуску. Signed packets читаються, але
автентичність підпису без ключа **не перевіряється**.

## Критерії

- **PASS**: обидва потоки присутні, MAVLink 2, один sender на потік,
  finite measurement/covariance; pose covariance — upper triangle 6×6 (21),
  speed — row-major 3×3 (9). Матриця symmetric PSD; singular nonzero PSD
  допускається. Спільний helper `vio_adapter.validate_covariance` відхиляє
  all-zero/NaN unknown uncertainty за integration policy. Це не твердження,
  що MAVLink забороняє позначати невідому covariance через NaN.
  Sender `usec` має строго зростати. Capture rate ≥ заданого порога,
  gaps ≤ max-gap.
- **FAIL**: порушення будь-якого перевіреного критерію, відсутній потік,
  covariance неправильного розміру/не finite/не PSD, timestamp reset або
  duplicate, недостатня частота чи завеликий gap.
- **NOT TESTED** (exit 2): зміст валідний, але немає двох різних capture
  timestamps для timing. Raw без arrival metadata не отримує timing PASS.
  PASS exit 0, FAIL exit 1; помилки читання також exit 1.

JSON окремо містить counts, sources, sender timestamps у **мікросекундах**,
capture timestamps у **секундах**, inter-arrival gaps, максимальний gap,
reset counters і dropout intervals для кожного типу. Observed rate =
`(count-1)/(last_capture-first_capture)`, не sender-declared rate.
Початкові/кінцеві gaps перевіряються лише в межах першого/останнього пакета
всього capture (включно з іншими типами). Нічого не доведено про тишу поза
цим вікном; включайте heartbeat або інший незалежний stream для покриття
кінцевої тиші. Довільна packet loss у capture може виглядати як VIO dropout.

Recovery PASS означає лише перший реально спостережений пакет після gap;
не стабільну навігацію, не EKF fusion/recovery. Загальний результат лишається
FAIL через порушення max-gap навіть після повернення потоку. Reset може бути
легітимним перезапуском VIO, але порушує обраний критерій безперервності.

## Як захопити реальний вихід VIO

ArduPilot **не зобов'язаний echo-ити вхідні vision packets**. FCU telemetry
output та `check_integration.py` не є доказом наявності VIO output.
Захоплюйте вхідний VIO MAVLink link **до FCU**: налаштуйте існуючий MAVLink
router/relay так, щоб він пересилав ті самі VIO bytes до FCU та на окремий
observer UDP endpoint. Не створюйте двох listeners на одному UDP port.
Запишіть router configuration, sender IDs, topology, clock source і команду
VIO. Копія на observer доводить прийом копії, не доставку/прийняття FCU.
Цей документ не припускає, що такий router уже налаштований.

Нижче мінімальний recorder для **вже налаштованого observer output** на
14556 (окремий вільний порт). Він не надсилає команд чи heartbeat і нічого
не змінює в FCU. Підключайте його перед запуском VIO; Ctrl-C закриває файл.
Використовуйте нове ім'я файла: `xb` захищає наявні докази від перезапису.

```bash
MAVLINK20=1 .venv/bin/python - <<'PY'
import struct
import time
from pymavlink import mavutil
link = mavutil.mavlink_connection('udpin:0.0.0.0:14556')
try:
    with open('integration/vio_rx.tlog', 'xb') as output:
        while True:
            msg = link.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            received_us = time.time_ns() // 1000
            if msg.get_type() == 'BAD_DATA':
                raise RuntimeError('Capture decode error; inspect link before validation')
            output.write(struct.pack('>Q', received_us & ~3))
            output.write(bytes(msg.get_msgbuf()))
            output.flush()
except KeyboardInterrupt:
    pass
finally:
    link.close()
PY
```

Timestamp тут — час user-space отримання/декодування на observer, включно з
можливим buffering/scheduling. Це **не** sensor capture time, kernel arrival
або FCU arrival. Для високої точності потрібен recorder із відповідною
instrumentation. Recorder/parser перевіряйте на фактичному transport;
його наведення тут не є live verification. Локальний `vio_rx.tlog` не
додавайте автоматично в commit.

Sender `usec` може бути boot-relative або Unix epoch: analyzer не вгадує
домен і не віднімає його від receiver time. Capture-to-EKF latency завжди
NOT TESTED без verified shared clock, acquisition timestamp і FCU fusion
evidence. EKF acceptance завжди NOT TESTED цим інструментом; потрібні
DataFlash innovations/fusion/rejection та узгоджена конфігурація EKF.
Валідна speed covariance на wire не доводить її використання ArduPilot 4.4.4.

Пов'язані матеріали: [README](README.md), [SITL](SITL.md),
[історичний synthetic covariance experiment](experiments/ekf444_covariance_20261104/REPORT.md).
