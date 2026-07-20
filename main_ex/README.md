# Керування висотою квадрокоптера через MAVLink і PD-регулятор

## 1. Призначення проєкту

Цей проєкт реалізує автономний сценарій польоту квадрокоптера ArduCopter у SITL:

1. підключення до автопілота через MAVLink;
2. перехід у режим `GUIDED`;
3. армування;
4. підйом до висоти 15 м;
5. утримання висоти 15 м протягом 18 с;
6. зниження до висоти 10 м;
7. утримання висоти 10 м протягом 18 с;
8. перехід у режим `LAND`;
9. очікування посадки й автоматичного дизармування.

Керування висотою виконується зовнішнім PD-регулятором у Python. Команда подається через MAVLink-повідомлення `SET_ATTITUDE_TARGET`, де орієнтація залишається горизонтальною, а керуючою величиною є параметр `thrust`.

Окремий скрипт будує графік поточної висоти в реальному часі та записує телеметрію у CSV.

---

## 2. Структура проєкту

```text
project/
├── main.py
├── setup.py
├── get_send.py
├── altitude_controller.py
├── landing.py
├── altitude_plotter.py
├── altitude_full_test.csv
└── README.md
```

### `main.py`

Головний сценарій польоту. Він послідовно:

- створює MAVLink-з'єднання;
- запитує потік `LOCAL_POSITION_NED` із частотою 20 Гц;
- створює PD-регулятор;
- переводить апарат у `GUIDED`;
- армує апарат;
- виконує етап 15 м;
- виконує етап 10 м;
- запускає посадку;
- у разі помилки або переривання намагається безпечно посадити апарат.

Параметри регулятора задаються тут:

```python
controller = AltitudePDController(
    kp=0.04,
    kd=0.14,
    hover_thrust=0.332,
    min_thrust=0.20,
    max_thrust=0.60,
)
```

### `setup.py`

Відповідає за початкову взаємодію з ArduPilot:

- `connect_to_autopilot()` створює MAVLink-з'єднання й очікує `HEARTBEAT`;
- `wait_command_ack()` очікує підтвердження MAVLink-команди;
- `set_guided_mode()` переводить апарат у режим `GUIDED`;
- `arm_vehicle()` надсилає команду армування.

### `get_send.py`

Містить низькорівневі функції отримання телеметрії та надсилання керування:

- `request_local_position_stream()` просить ArduPilot публікувати `LOCAL_POSITION_NED` із заданою частотою;
- `read_vertical_state()` читає поточну висоту та вертикальну швидкість;
- `send_thrust()` надсилає `SET_ATTITUDE_TARGET`.

У `LOCAL_POSITION_NED` вісь `Z` спрямована вниз, тому значення перетворюються так:

```python
altitude_m = -message.z
vertical_speed_up_m_s = -message.vz
```

### `altitude_controller.py`

Містить сам PD-регулятор і цикл керування висотою.

Клас `AltitudePDController` обчислює thrust:

```text
error = target_altitude - current_altitude
P = kp × error
D = -kd × vertical_speed_up
thrust = hover_thrust + P + D
```

Після цього thrust обмежується діапазоном:

```text
min_thrust ≤ thrust ≤ max_thrust
```

Функція `reach_and_hold_altitude()`:

- працює приблизно на 20 Гц;
- читає висоту і вертикальну швидкість;
- обчислює thrust;
- надсилає керування;
- перевіряє входження у стабільну область;
- запускає таймер утримання;
- скидає таймер, якщо дрон виходить за межі допуску;
- завершує етап після безперервного утримання протягом 18 с.

Поточні параметри етапу:

```python
CONTROL_FREQUENCY_HZ = 20.0
ALTITUDE_TOLERANCE_M = 0.25
VERTICAL_SPEED_TOLERANCE_M_S = 0.20
HOLD_DURATION_S = 18.0
STAGE_TIMEOUT_S = 90.0
```

### `landing.py`

Відповідає за безпечне завершення польоту:

- переводить апарат у режим `LAND`;
- перевіряє підтвердження режиму;
- очікує посадку;
- очікує автоматичне дизармування.

### `altitude_plotter.py`

Окремий процес для візуалізації польоту. Він:

- підключається до другого MAVLink UDP-порту;
- читає `LOCAL_POSITION_NED`;
- будує графік висоти в реальному часі;
- показує поточну висоту й вертикальну швидкість;
- зберігає дані в CSV.

### `altitude_full_test.csv`

CSV-файл із записаною телеметрією польоту. Містить:

```text
wall_time_iso
elapsed_s
altitude_m
vertical_speed_up_m_s
x_m
y_m
z_ned_m
```

---

## 3. Загальна архітектура

```text
Gazebo Harmonic
      ↓
ArduPilot SITL
      ↓
MAVProxy
      ├── UDP 14550 → main.py
      └── UDP 14551 → altitude_plotter.py

main.py
  ↓
setup.py
  ↓
GUIDED + ARM
  ↓
get_send.py
  ↓
LOCAL_POSITION_NED
  ↓
altitude_controller.py
  ↓
PD-регулятор
  ↓
SET_ATTITUDE_TARGET
  ↓
ArduPilot
  ↓
двигуни моделі Iris
```

Графік працює незалежно від керуючого скрипта. Він отримує копію MAVLink-потоку через порт `14551`, тому не забирає повідомлення у `main.py`, який використовує `14550`.

---

## 4. Як працює керування висотою

### Помилка висоти

```text
error = target_altitude - current_altitude
```

Якщо поточна висота менша за цільову, помилка додатна, тому P-складова збільшує thrust.

Якщо дрон вище цільової висоти, помилка від'ємна, тому thrust зменшується.

### P-складова

```text
P = kp × error
```

Вона визначає, наскільки сильно контролер реагує на відхилення від цільової висоти.

### D-складова

```text
D = -kd × vertical_speed_up
```

D-складова гальмує вертикальний рух:

- при швидкому підйомі вона зменшує thrust;
- при швидкому зниженні вона збільшує thrust.

Це зменшує переліт через setpoint і робить наближення до висоти плавнішим.

### Базова тяга

```text
hover_thrust = 0.332
```

Це приблизне значення тяги, потрібне для зависання Iris без вертикального прискорення.

Підсумкове керування:

```text
thrust = hover_thrust + P + D
```

### Стабільна область

Висота вважається стабільною, коли одночасно виконуються дві умови:

```text
|altitude_error| ≤ 0.25 м
|vertical_speed| ≤ 0.20 м/с
```

Після входження у стабільну область запускається таймер 18 с. Якщо хоча б одна умова порушується, таймер скидається.

---

## 5. Вимоги

Середовище, на якому перевірявся проєкт:

- Windows 11;
- WSL2 Ubuntu;
- Python 3;
- ArduPilot SITL;
- MAVProxy;
- Gazebo Harmonic;
- ArduPilot Gazebo plugin;
- `pymavlink`;
- `matplotlib`.

Python-залежності:

```bash
pip install pymavlink matplotlib
```

У проєкті використовується віртуальне середовище:

```bash
source ~/venv-ardupilot/bin/activate
```

---

## 6. Підготовка ArduPilot

Для керування параметром `thrust` через `SET_ATTITUDE_TARGET` необхідно встановити:

```text
GUID_OPTIONS = 8
```

У MAVProxy:

```text
param set GUID_OPTIONS 8
param show GUID_OPTIONS
```

Для Iris також перевіряються:

```text
param show FRAME_CLASS
param show FRAME_TYPE
```

Очікувані значення:

```text
FRAME_CLASS 1
FRAME_TYPE 1
GUID_OPTIONS 8
```

---

## 7. Запуск проєкту

Для повного запуску потрібні чотири термінали.

### Термінал 1: Gazebo Harmonic

```bash
source ~/drone_sim_runners/env.sh

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

gz sim -v4 -r "$ARDUPILOT_GAZEBO_DIR/worlds/iris_runway.sdf"
```

Після запуску має відкритися Gazebo зі світом `iris_runway` та моделлю Iris.

### Термінал 2: ArduPilot SITL і MAVProxy

```bash
source ~/venv-ardupilot/bin/activate
cd ~/ardupilot

./Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --model JSON \
  --out 127.0.0.1:14550 \
  --out 127.0.0.1:14551
```

Після завантаження у MAVProxy виконати:

```text
param show FRAME_CLASS
param show FRAME_TYPE
param show GUID_OPTIONS
output
```

MAVProxy повинен показати два UDP-виходи:

```text
127.0.0.1:14550
127.0.0.1:14551
```

### Термінал 3: графік висоти

Перейти до каталогу проєкту:

```bash
cd /mnt/c/Users/User/Projects/realsans_camera
source ~/venv-ardupilot/bin/activate
```

Видалити попередній CSV, якщо потрібен чистий запис:

```bash
rm -f altitude_full_test.csv
```

Запустити графік:

```bash
python3 -u altitude_plotter.py \
  --connection udpin:0.0.0.0:14551 \
  --window 180 \
  --csv altitude_full_test.csv
```

Параметри:

- `--connection` задає MAVLink-порт графіка;
- `--window 180` показує останні 180 с;
- `--csv` задає файл запису телеметрії.

### Термінал 4: основний сценарій

```bash
cd /mnt/c/Users/User/Projects/realsans_camera
source ~/venv-ardupilot/bin/activate
```

Перевірка синтаксису:

```bash
python3 -m py_compile \
  main.py \
  setup.py \
  get_send.py \
  altitude_controller.py \
  landing.py \
  altitude_plotter.py
```

Запуск:

```bash
python3 -u main.py
```

Якщо копія головного сценарію називається `main_ex.py`:

```bash
python3 -u main_ex.py
```

---

## 8. Очікувана послідовність виконання

```text
Attempting to connect to udpin:0.0.0.0:14550
Waiting for HEARTBEAT message...
ArduPilot is found
Requested LOCAL_POSITION_NED at 20.0 Hz
GUIDED mode has been confirmed
The ARM command was accepted
Stage 1: climb to 15 m and hold
Altitude 15.00 m was held for 18.0 s
Stage 2: descend to 10 m and hold
Altitude 10.00 m was held for 18.0 s
Altitude sequence completed. Starting landing
LAND mode confirmed
Vehicle landed and disarmed
Mission completed successfully
MAVLink connection closed
```

Під час кожного етапу приблизно чотири рази на секунду друкується:

```text
target=15.00 m | alt=14.92 m | speed=+0.02 m/s | error=+0.08 m | P=+0.003 | D=-0.003 | thrust=0.333 | stable 7.11/18.00 s
```

---

## 9. Обробка помилок

`main.py` використовує `try / except / finally`.

Якщо виникає помилка після армування:

1. повідомлення про помилку друкується у консоль;
2. викликається `land_and_wait()`;
3. апарат переводиться у `LAND`;
4. система очікує дизармування;
5. MAVLink-з'єднання закривається.

Так само обробляється `Ctrl+C`.

Це не гарантує фізичну безпеку реального апарата, але для SITL забезпечує контрольоване завершення сценарію замість покинутого armed-дрона, який висить у симуляції й розмірковує про сенс існування.

---

## 10. Основні параметри для налаштування

### У `main.py`

```python
kp=0.04
kd=0.14
hover_thrust=0.332
min_thrust=0.20
max_thrust=0.60
```

### У `altitude_controller.py`

```python
CONTROL_FREQUENCY_HZ = 20.0
ALTITUDE_TOLERANCE_M = 0.25
VERTICAL_SPEED_TOLERANCE_M_S = 0.20
HOLD_DURATION_S = 18.0
STAGE_TIMEOUT_S = 90.0
```

### Цільові висоти у `main.py`

```python
target_altitude=15.0
target_altitude=10.0
```

---

## 11. Типові проблеми

### Немає `HEARTBEAT`

```text
Heartbeat timed out
```

Перевірити:

- чи запущений SITL;
- чи існує вихід `14550`;
- чи не зайнятий порт іншим процесом;
- чи правильно вказано `udpin:0.0.0.0:14550`.

### Немає `LOCAL_POSITION_NED`

```text
LOCAL_POSITION_NED was not received
```

Перевірити:

- чи викликана `request_local_position_stream()`;
- чи працює SITL;
- чи не читає той самий UDP-порт інший процес;
- чи отримано підтвердження MAVLink-з'єднання.

### Дрон не реагує на thrust

Перевірити:

```text
param show GUID_OPTIONS
```

Має бути:

```text
GUID_OPTIONS 8
```

### Етап завершується через timeout

```text
Could not reach and hold ... within ... seconds
```

`STAGE_TIMEOUT_S` рахується від початку етапу й включає:

- набір або зниження висоти;
- входження у стабільну область;
- повний час утримання.

Тому він має бути більшим за суму часу переходу та `HOLD_DURATION_S`.

### Графік не отримує дані

Перевірити, що SITL запущено з другим виходом:

```bash
--out 127.0.0.1:14551
```

І що графік використовує саме його:

```bash
--connection udpin:0.0.0.0:14551
```

---

## 12. Завершення роботи

Рекомендований порядок:

1. дочекатися завершення `main.py` і дизармування;
2. закрити вікно графіка;
3. зупинити SITL через `Ctrl+C`;
4. закрити Gazebo.

Якщо сценарій потрібно зупинити достроково, натиснути `Ctrl+C` у терміналі `main.py`. Скрипт спробує перейти у `LAND` і дочекатися дизармування.
