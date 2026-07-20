# Horizontal Attitude Control

Проєкт демонструє керування квадрокоптером ArduPilot у SITL через `pymavlink` і команду `SET_ATTITUDE_TARGET`.

Сценарій польоту:

1. підключення до ArduPilot через MAVLink;
2. запит телеметрії `LOCAL_POSITION_NED` та `ATTITUDE`;
3. перехід у режим `GUIDED`;
4. армінг;
5. підйом до висоти `10 м`;
6. стабілізація та утримання висоти;
7. політ на `10 м` уперед через керування кутом `pitch`;
8. одночасне утримання висоти через керування `thrust`;
9. утримання кінцевої позиції;
10. перехід у `LAND` та очікування автоматичного дизармінгу.


## Структура проєкту

```text
horizontal_attitude_control/
├── main.py
├── config.py
├── flight_controller.py
├── pid_controller.py
├── telemetry.py
├── mavlink_connection.py
├── attitude_command.py
├── flight_logger.py
├── live_plot.py
├── requirements.txt
└── flight_log.csv
```

## Призначення файлів

### `main.py`

Точка входу в програму.

Файл:

- створює MAVLink-з'єднання;
- запитує потрібні потоки телеметрії;
- створює об'єкти `TelemetryReader`, `FlightLogger` і `DroneFlightController`;
- переводить дрон у `GUIDED`;
- виконує армінг;
- запускає підйом до заданої висоти;
- запускає рух уперед;
- після завершення місії виконує посадку;
- при помилці або перериванні також намагається безпечно посадити дрон.

Основна послідовність:

```python
controller.hold_altitude(TARGET_ALTITUDE_M)
controller.move_forward(TARGET_ALTITUDE_M, FORWARD_DISTANCE_M)
land_and_wait(connection, LAND_TIMEOUT_S)
```

### `config.py`

Містить усі основні параметри проєкту.

Основні параметри місії:

```python
TARGET_ALTITUDE_M = 10.0
FORWARD_DISTANCE_M = 10.0
ALTITUDE_HOLD_DURATION_S = 8.0
POSITION_HOLD_DURATION_S = 12.0
```

Частоти:

```python
TELEMETRY_FREQUENCY_HZ = 20.0
CONTROL_FREQUENCY_HZ = 20.0
```

Параметри вертикального регулятора:

```python
ALTITUDE_KP = 0.04
ALTITUDE_KI = 0.0
ALTITUDE_KD = 0.14
```

Параметри горизонтального регулятора:

```python
FORWARD_KP = 0.045
FORWARD_KI = 0.0
FORWARD_KD = 0.12
```

Обмеження керування:

```python
HOVER_THRUST = 0.332
MIN_THRUST = 0.20
MAX_THRUST = 0.60
MAX_PITCH_DEG = 12.0
```

### `pid_controller.py`

Містить універсальний клас `PIDController`.

Регулятор обчислює:

```text
output = P + I + D
```

де:

```text
P = Kp × error
I = Ki × integral(error)
D = -Kd × measured_speed
```

У поточних налаштуваннях `Ki = 0`, тому обидва регулятори фактично працюють як PD-регулятори, але клас залишений універсальним.

Метод `clamp()` обмежує вихід регулятора допустимим діапазоном.

### `telemetry.py`

Клас `TelemetryReader` читає:

- `LOCAL_POSITION_NED`;
- `ATTITUDE`.

Повертається словник стану:

```python
{
    "north": ...,
    "east": ...,
    "altitude": ...,
    "velocity_north": ...,
    "velocity_east": ...,
    "velocity_up": ...,
    "roll": ...,
    "pitch": ...,
    "yaw": ...,
}
```

`LOCAL_POSITION_NED` використовує систему координат NED:

```text
X = North
Y = East
Z = Down
```

Тому висота й вертикальна швидкість перетворюються так:

```python
altitude = -position.z
velocity_up = -position.vz
```

### `mavlink_connection.py`

Містить функції низькорівневої взаємодії з ArduPilot:

- `connect_to_autopilot()`;
- `request_message_interval()`;
- `request_telemetry()`;
- `set_mode()`;
- `arm_vehicle()`;
- `land_and_wait()`.

З'єднання виконується через:

```python
udpin:0.0.0.0:14550
```

Телеметрія `LOCAL_POSITION_NED` і `ATTITUDE` запитується з частотою `20 Гц`.

### `attitude_command.py`

Відповідає за формування `SET_ATTITUDE_TARGET`.

Функція `euler_to_quaternion()` переводить кути `roll`, `pitch`, `yaw` у quaternion.

Функція `send_attitude_target()` надсилає:

- quaternion орієнтації;
- thrust;
- нульові body rates.

Використовується `type_mask = 7`, тому body roll rate, pitch rate і yaw rate ігноруються, а орієнтація задається quaternion-ом.

Функція `compensate_tilt_thrust()` компенсує зменшення вертикальної складової тяги під час нахилу:

```text
thrust_compensated = vertical_thrust / (cos(roll) × cos(pitch))
```

Без цієї компенсації дрон під час нахилу вперед міг би втрачати висоту.

### `flight_controller.py`

Основний модуль керування польотом.

Клас `DroneFlightController` містить два незалежні регулятори:

```text
altitude PID → thrust
forward PID → pitch
```

#### Підйом і утримання висоти

Метод:

```python
hold_altitude(target_altitude)
```

На кожній ітерації:

1. читає поточну висоту та вертикальну швидкість;
2. обчислює помилку висоти;
3. вертикальний PID формує поправку до `HOVER_THRUST`;
4. надсилається `SET_ATTITUDE_TARGET` із нульовими `roll` і `pitch`;
5. перевіряється входження у стабільну область;
6. після безперервного утримання протягом `ALTITUDE_HOLD_DURATION_S` етап завершується.

Стабільний стан визначається умовами:

```text
|altitude_error| ≤ ALTITUDE_TOLERANCE_M
|vertical_speed| ≤ VERTICAL_SPEED_TOLERANCE_M_S
```

#### Рух уперед

Метод:

```python
move_forward(target_altitude, distance_m)
```

На початку етапу запам'ятовуються:

- початкові координати `north` і `east`;
- початковий yaw.

Поточне переміщення вперед обчислюється проєкцією глобального зміщення на напрям носа дрона:

```python
forward_position = cos(yaw) * delta_north + sin(yaw) * delta_east
```

Швидкість уперед:

```python
forward_speed = cos(yaw) * velocity_north + sin(yaw) * velocity_east
```

Горизонтальний регулятор формує команду pitch:

```python
pitch_command = -forward_pid.calculate(forward_error, forward_speed, dt)
```

Негативний pitch використовується для нахилу вперед у прийнятій системі знаків.

Паралельно вертикальний регулятор продовжує утримувати висоту через thrust.

Етап завершується тільки тоді, коли одночасно виконані умови:

```text
похибка позиції мала;
горизонтальна швидкість мала;
похибка висоти мала;
вертикальна швидкість мала;
стан утримується заданий час.
```

### `flight_logger.py`

Записує телеметрію в `flight_log.csv`.

Колонки:

```text
time_s
stage
target_altitude_m
altitude_m
target_forward_m
forward_m
forward_speed_m_s
vertical_speed_m_s
pitch_command_deg
thrust
```

Поле `stage` має значення:

```text
altitude
forward
```

CSV використовується для аналізу польоту та побудови графіків.

### `live_plot.py`

Читає `flight_log.csv` під час польоту і показує два графіки.

Верхній графік:

```text
Altitude setpoint
Current altitude
```

Нижній графік:

```text
Forward setpoint
Current forward position
```

Графік оновлюється приблизно кожні `250 мс`.

Він не читає MAVLink напряму, тому не конкурує з основним процесом за один UDP-потік.

### `flight_log.csv`

Архів уже містить приклад успішного польоту.

За записаними даними:

```text
етап altitude: приблизно 0.19 м → 9.93 м;
етап forward: приблизно 0.00 м → 10.007 м;
кінцева висота: близько 10.00 м;
кінцева горизонтальна швидкість: близько 0 м/с;
кінцева вертикальна швидкість: близько 0 м/с;
кінцевий thrust: близько 0.332.
```

Це показує, що дрон досяг заданої висоти, пролетів приблизно `10 м` уперед і стабілізувався біля цільової позиції.

## Загальний потік даних

```text
ArduPilot SITL
    ↓
LOCAL_POSITION_NED + ATTITUDE
    ↓
TelemetryReader
    ↓
DroneFlightController
    ├── altitude PID → thrust
    └── forward PID → pitch
    ↓
SET_ATTITUDE_TARGET
    ↓
ArduPilot attitude controller
    ↓
Gazebo Iris model
```

Паралельно:

```text
DroneFlightController
    ↓
FlightLogger
    ↓
flight_log.csv
    ↓
LiveFlightPlot
```

## Вимоги

- Ubuntu або WSL2;
- Python 3;
- ArduPilot SITL;
- MAVProxy;
- Gazebo Harmonic;
- ArduPilot Gazebo plugin;
- Python-пакети `pymavlink` і `matplotlib`.

Встановлення Python-залежностей:

```bash
cd /mnt/c/Users/User/Projects/horizontal_attitude_control
source ~/venv-ardupilot/bin/activate
pip install -r requirements.txt
```

## Параметр ArduPilot

Для керування thrust через `SET_ATTITUDE_TARGET` потрібно:

```text
GUID_OPTIONS = 8
```

Перевірка в MAVProxy:

```text
param show GUID_OPTIONS
```

Встановлення:

```text
param set GUID_OPTIONS 8
```

Також варто перевірити конфігурацію рами:

```text
param show FRAME_CLASS
param show FRAME_TYPE
```

Для Iris очікується:

```text
FRAME_CLASS 1
FRAME_TYPE 1
```

## Запуск проєкту

Проєкт запускається у чотирьох терміналах.

### Термінал 1: Gazebo

```bash
source ~/drone_sim_runners/env.sh
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"
gz sim -v4 -r "$ARDUPILOT_GAZEBO_DIR/worlds/iris_runway.sdf"
```

### Термінал 2: ArduPilot SITL

```bash
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot

./Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --model JSON \
  --out 127.0.0.1:14550
```

Після запуску в MAVProxy:

```text
param show FRAME_CLASS
param show FRAME_TYPE
param show GUID_OPTIONS
```

### Термінал 3: графік

```bash
cd /mnt/c/Users/User/Projects/horizontal_attitude_control
source ~/venv-ardupilot/bin/activate
rm -f flight_log.csv
python3 -u live_plot.py
```

Спочатку вікно може бути порожнім. Дані з'являться після запуску `main.py`.

### Термінал 4: керування польотом

```bash
cd /mnt/c/Users/User/Projects/horizontal_attitude_control
source ~/venv-ardupilot/bin/activate

python3 -m py_compile \
  main.py \
  config.py \
  pid_controller.py \
  telemetry.py \
  mavlink_connection.py \
  attitude_command.py \
  flight_logger.py \
  flight_controller.py \
  live_plot.py

python3 -u main.py
```

## Очікуваний порядок виконання

```text
Connecting to udpin:0.0.0.0:14550
Waiting for heartbeat
Connected
Telemetry requested at 20.0 Hz
GUIDED mode confirmed
Vehicle armed
Climbing to 10.0 m
Moving forward 10.0 m
Mission completed
LAND mode confirmed
Vehicle landed and disarmed
```

Під час підйому виводиться приблизно така діагностика:

```text
alt_target=10.00 alt=8.75 vz=+0.42 thrust=0.323 hold=0.0/8.0
```

Під час руху вперед:

```text
x_target=10.00 x=7.40 vx=+0.82 alt=10.01 pitch=-4.20 hold=0.0/12.0
```

## Безпечне завершення

При `Ctrl+C` або винятку `main.py` переходить у блок `finally` і намагається виконати:

```python
land_and_wait(connection, LAND_TIMEOUT_S)
```

Тому помилка контролера не повинна залишити дрон у повітрі без команди посадки.

## Типові проблеми

### Не приходить heartbeat

Перевірити:

```text
SITL запущено;
порт 14550 додано через --out;
main.py запускається після SITL.
```

### `GUIDED mode was not confirmed`

Перевірити, чи підтримується режим `GUIDED` і чи ArduPilot завершив ініціалізацію.

### Дрон не реагує на thrust

Перевірити:

```text
GUID_OPTIONS = 8
режим GUIDED активний
дрон армований
SET_ATTITUDE_TARGET надсилається циклічно
```

### Дрон летить назад

Змінити знак команди pitch у `flight_controller.py`:

```python
pitch_command = -self.forward_pid.calculate(...)
```

на протилежний лише після перевірки фактичної орієнтації моделі. Система координат, як завжди, робить вигляд, що знак очевидний, хоча це ніколи не так.

### Дрон втрачає висоту під час нахилу

Перевірити роботу:

```python
compensate_tilt_thrust()
```

Також можна зменшити `MAX_PITCH_DEG` або повторно підібрати коефіцієнти вертикального регулятора.

### Спрацьовує timeout

Збільшити:

```python
ALTITUDE_STAGE_TIMEOUT_S
FORWARD_STAGE_TIMEOUT_S
```

Таймаут етапу має бути більшим за час виходу на ціль плюс час стабільного утримання.

### Графік порожній

Перевірити:

```bash
ls -lh flight_log.csv
head flight_log.csv
```

`live_plot.py` і `main.py` повинні запускатися з однієї директорії, щоб обидва використовували той самий `flight_log.csv`.

## Результат

Проєкт реалізує двоканальне керування:

```text
висота → thrust
горизонтальна позиція → pitch
```

Команда `SET_ATTITUDE_TARGET` одночасно передає бажану орієнтацію і тягу. Завдяки цьому дрон може рухатися вперед, не відмовляючись від висоти заради драматичного падіння в пейзаж Gazebo.
