# Waveshare 10 DOF ROS IMU (A) — примеры и утилиты

Скрипты из официального архива Waveshare для работы с модулем по **UART** и **Bluetooth**.

Документация производителя: https://www.waveshare.com/wiki/10_DOF_ROS_IMU_(A)

---

## Структура каталога

```
scripts/
├── README.md
├── set_direction.py              # матрица ориентации PCB на корпусе
├── 3d_model/                       # STEP-модели корпуса и платы
├── uart_example_code/              # чтение данных по UART
│   ├── imu_uart.py                 # вывод в консоль
│   └── imu_uart_gui.py             # 3D-визуализация ориентации (OpenGL)
├── init_acc_code/                 # калибровка акселерометра (6 граней)
│   └── imu_init.py
├── init_mag_code/                 # калибровка магнитометра
│   └── imu_init.py
└── ble_example_code/
    └── Linux/
        └── imu_ble.py              # то же по Bluetooth (MAC-адрес)
```

---

## Подключение

| Параметр | Значение |
|----------|----------|
| Скорость | **115200**, 8N1 |
| Питание | 3.3–5 V |

Обычно CH340: `/dev/ttyUSB0`. Пользователь должен быть в группе `dialout`:

```bash
sudo usermod -aG dialout $USER
```

---

## Зависимости

### UART-скрипты (все `imu_*.py`, `set_direction.py`)

```bash
sudo apt install python3-serial python3-numpy
```

### GUI (`imu_uart_gui.py`)

```bash
sudo apt install python3-pygame python3-opengl
```

### Bluetooth (`ble_example_code/Linux/imu_ble.py`)

```bash
sudo apt install bluez libbluetooth-dev
```

---

## Порт

```python
ser_port = "/dev/ttyUSB0"
```

Файлы с `ser_port`:

- `uart_example_code/imu_uart.py`
- `uart_example_code/imu_uart_gui.py`
- `init_acc _code/imu_init.py`
- `init_mag _code/imu_init.py`
- `set_direction.py`

---

## Рекомендуемый порядок работы

```
1. Проверка связи      -> uart_example_code/imu_uart.py
2. Калибровка accel    -> init_acc _code/imu_init.py
3. Калибровка mag      -> init_mag _code/imu_init.py
4. Ориентация на PCB   -> set_direction.py (если модуль крепится нестандартно)
5. ROS 2               -> пакет imu в корне репозитория
```

Калибровки сохраняются **в прошивке модуля** — повторять после каждого запуска не нужно.

---

## Скрипты

### `uart_example_code/imu_uart.py`

Минимальный пример: настройка модуля, включение автоотчёта, вывод данных в терминал (углы, кватернион, ускорение и т.д.).

```bash
cd uart_example_code
python3 imu_uart.py
```


### `uart_example_code/imu_uart_gui.py`

То же но с интерфейсом с gui.

```bash
cd uart_example_code
python3 imu_uart_gui.py
```

---

### `init_acc _code/imu_init.py`

**Калибровка акселерометра** (команда `0x17`, 6 граней).

1. Положите модуль на грань и дождитесь, пока индикатор перестанет мигать (грань принята).
2. Повторите для всех 6 граней.
3. Нажмите **Enter** в терминале для завершения.

```bash
cd "init_acc _code"
python3 imu_init.py
```

После калибровки модуль в покое: модуль ускорения |g| ≈ 9.8 m/s².

---

### `init_mag _code/imu_init.py`

**Калибровка магнитометра** (команды `0x32` / `0x04`).

В течение ~10 с вращайте модуль: ось Z вертикально и горизонтально, полный оборот в плоскости.

```bash
cd "init_mag _code"
python3 imu_init.py
```

Датчик вдали от металла, моторов и сильных полей.

---

### `set_direction.py`

Задание **матрицы установки PCB** (команда `0x20`): как акселерометр и магнитометр ориентированы на корпусе робота.  
По умолчанию задана одна из типовых матриц — при необходимости отредактируйте `accMatrix` / `magMatrix` в файле.

```bash
python3 set_direction.py
```

Используйте, если оси IMU на роботе не совпадают с осями корпуса.

---

### `ble_example_code/Linux/imu_ble.py`

Тот же протокол по **Bluetooth**. Нужен MAC модуля (nRF Connect / LightBlue).

```bash
cd ble_example_code/Linux
python3 imu_ble.py 70:53:B2:02:20:02
```

Замените MAC на свой. После подключения в течение 30 с нужна команда удержания сессии (`0x29`) — в скрипте это уже реализовано.

---

## ROS 2 (основной пакет)

Драйвер для робота — не в `scripts/`, а в `../`:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch imu imu.launch.py port_name:=/dev/ttyUSB0
ros2 topic echo /imu/data
```

С RViz:

```bash
ros2 launch imu imu_view.launch.py port_name:=/dev/ttyUSB0
```

При старте нода выполняет `Cmd_05` (обнуление Z), но **не** полную калибровку accel/mag — её делают скриптами выше.

---

## Устранение неполадок

| Симптом | Что проверить |
|---------|----------------|
| Нет данных в UART | TX↔RX, GND, `ser_port`, права `dialout` |
| `ModuleNotFoundError: OpenGL` | `sudo apt install python3-opengl python3-pygame` |
| `could not open port` | Занят другим процессом (`imu_node`, другой скрипт) |
| Данные «плывут» по yaw | Сделать `init_mag _code/imu_init.py` |
| Неверный |g| после покоя | Сделать `init_acc _code/imu_init.py` |
| Оси не совпадают с роботом | Настроить `set_direction.py` |

---

## 3D-модели

`3d_model/` — файлы STEP для CAD (корпус и плата). К запуску скриптов не относятся.

---