"""Общее логирование для скриптов IMU."""
import logging
import sys


def setup_logger(name='imu'):
    log = logging.getLogger(name)
    if log.handlers:
        return log
    log.setLevel(logging.INFO)
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter(
        '[%(asctime)s] %(levelname)s: %(message)s',
        datefmt='%H:%M:%S',
    ))
    log.addHandler(handler)
    return log


def step(log, message):
    log.info('>>> %s', message)


def open_serial(log, port, baudrate=115200, timeout=2):
    import serial
    log.info('Подключение к %s (%d бод)...', port, baudrate)
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
    except serial.SerialException as exc:
        log.error('Не удалось открыть порт %s: %s', port, exc)
        raise SystemExit(1) from exc
    log.info('Порт %s открыт', port)
    return ser
