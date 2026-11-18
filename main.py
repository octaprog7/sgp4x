# SPDX-License-Identifier: GPL-3.0-or-later
# micropython

from micropython import const
from time import sleep_ms, ticks_ms, ticks_diff
from machine import I2C, SoftI2C, Pin
from sgp4Xmod import SGP4X
from sensor_pack_2.bus_service import I2cAdapter
from sen_gas_index import GasIndexAlgorithm, ALGORITHM_TYPE_VOC, ALGORITHM_TYPE_NOX

# период опроса
POLLING_PERIOD_MS = const(1000)

def describe_air_quality(voc_index: int, nox_index: int | None = None) -> tuple[str, str]:
    """
    Принимает обработанные(!) индексы VOC (0..500) и NOx (0..500 или None для SGP40).
    Возвращает кортеж из двух строк с понятной для пользователя оценкой(!) качества воздуха.
    """
    # Оценка VOC (бытовые(!) запахи, свежесть, духота)
    if voc_index <= 0:
        voc_msg = "VOC: sensor warming up..."
    elif voc_index < 80:
        voc_msg = "VOC: very fresh air"
    elif voc_index <= 120:
        voc_msg = "VOC: normal air quality"
    elif voc_index <= 200:
        voc_msg = "VOC: slight odors, consider ventilating"
    elif voc_index <= 350:
        voc_msg = "VOC: stuffy air / chemicals, ventilate now"
    else:
        voc_msg = "VOC: heavy odor pollution!"

    # Оценка NOx (газовая плита, дым, выхлопные газы)
    if nox_index is None:
        nox_msg = "NOx: not supported (SGP40)"
    elif nox_index <= 0:
        nox_msg = "NOx: sensor warming up..."
    elif nox_index <= 2:
        nox_msg = "NOx: clean, no smoke or fumes"
    elif nox_index <= 20:
        nox_msg = "NOx: slight trace of combustion / exhaust"
    elif nox_index <= 100:
        nox_msg = "NOx: smoke or exhaust detected, turn on hood"
    else:
        nox_msg = "NOx: hazardous smoke or combustion gases!"

    return voc_msg, nox_msg


if __name__ == '__main__':
    # Установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(id=1, scl=Pin(27), sda=Pin(26), freq=400_000)  # on Arduino Nano RP2040 Connect and Pico W tested!
    # i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)
    sda_pin = 8  # Replace with your SDA pin
    scl_pin = 9  # Replace with your SCL pin
    # ESP32-C3 не имеет аппаратного I2C. Там, где он есть, используйте I2C вместо SoftI2C!
    bus = SoftI2C(sda=Pin(sda_pin), scl=Pin(scl_pin), freq=400_000)
    # bus = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)
    adaptor = I2cAdapter(bus)
    sensor = SGP4X(adapter=adaptor, address=0x59, check_crc=True, sensor_id = 0)

    # Инициализация алгоритмов расчёта индексов (интервал 1.0 с)
    voc_algo = GasIndexAlgorithm(algorithm_type=ALGORITHM_TYPE_VOC, sampling_interval=1.0)
    nox_algo = GasIndexAlgorithm(algorithm_type=ALGORITHM_TYPE_NOX, sampling_interval=1.0)

    # 3.6 После подачи питания датчик переходит в idle mode максимум через 0.6 мс
    sleep_ms(10)

    try:
        sen_id = sensor.get_id()
        print(f"Serial number: {sen_id}")

        test_result = sensor.execute_self_test()
        print(f"Self-test result: 0x{test_result:x}")
        if 0xD400 == test_result:
            print("All tests passed successfully!")
        else:
            print("One or more tests failed!")
        sleep_ms(100)

        # Кондиционирование (только для SGP41, не дольше 10 секунд, нагреватель после не отключаю!)
        if 1 == sensor.get_sensor_id():
            print("Conditioning SGP41 (10 s)...")
            for sec in range(10):
                t0 = ticks_ms()
                cond_raw = sensor.execute_conditioning()
                # Во время кондиционирования SGP41 уже отдает достаточно точный RAW_VOC,
                # поэтому включаю в работу VOC-фильтр
                voc_idx = voc_algo.get_gas_index(cond_raw.VOC)
                print(f"[{sec + 1}/10 sec.] Conditioning VOC raw: {cond_raw.VOC} -> VOC Index: {voc_idx}")
                elapsed = ticks_diff(ticks_ms(), t0)
                if elapsed < POLLING_PERIOD_MS:
                    sleep_ms(POLLING_PERIOD_MS - elapsed)
            print("Conditioning complete.")

        if 0 == sensor.get_sensor_id():
            print("SGP40 sensor.")

        print("Starting continuous measurement (first 45 sec. warm-up returns index 0)...")
        for _ in range(1000):
            t0 = ticks_ms()
            raw = sensor.measure_raw_signal()

            voc_idx = voc_algo.get_gas_index(raw.VOC)
            nox_idx = nox_algo.get_gas_index(raw.NOx) if raw.NOx is not None else None

            if 0 == voc_idx:
                # Остаток времени прогрева
                rem_sec = max(0, int(46 - voc_algo.uptime))
                # print(f"[Warm-up: {rem_sec:2d} sec. left] VOC raw: {raw.VOC}| NOx raw: {raw.NOx}" if raw.NOx is not None else "")
                print(f"[Warm-up: {rem_sec:2d} sec. left] VOC raw: {raw.VOC}; NOx raw: {raw.NOx}")
            else:
                aq = describe_air_quality(voc_idx, nox_idx)
                if nox_idx is not None:
                    print(f"VOC raw: {raw.VOC} -> Index: {voc_idx} | NOx raw: {raw.NOx} -> Index: {nox_idx} | {aq}")
                else:
                    print(f"VOC raw: {raw.VOC} -> Index: {voc_idx} | {aq}")

            # период опроса POLLING_PERIOD_MS
            elapsed = ticks_diff(ticks_ms(), t0)
            if elapsed < POLLING_PERIOD_MS:
                sleep_ms(POLLING_PERIOD_MS - elapsed)

    finally:
        # отключаю нагреватель при завершении
        sensor.turn_heater_off()
        print("Heater turned off.")
