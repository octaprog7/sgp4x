# micropython
# MIT license

from time import sleep_ms, ticks_ms, ticks_diff
from machine import I2C, SoftI2C, Pin
from sgp4Xmod import SGP4X
from sensor_pack_2.bus_service import I2cAdapter
from sen_gas_index import GasIndexAlgorithm, ALGORITHM_TYPE_VOC, ALGORITHM_TYPE_NOX


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(id=1, scl=Pin(27), sda=Pin(26), freq=400_000)  # on Arduino Nano RP2040 Connect and Pico W tested!
    # i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)
    sda_pin = 8  # Replace with your SDA pin
    scl_pin = 9  # Replace with your SCL pin
    # ESP32-C3 не имеет аппаратного I2C. Там, где он есть, используйте I2C вместо SoftI2C!
    # bus = SoftI2C(sda=Pin(sda_pin), scl=Pin(scl_pin), freq=400_000)
    bus = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)
    adaptor = I2cAdapter(bus)
    sensor = SGP4X(adapter=adaptor, address=0x59, check_crc=True, sensor_id = 0)

    # Инициализация алгоритмов расчёта индексов (строгий интервал 1.0 с)
    voc_algo = GasIndexAlgorithm(ALGORITHM_TYPE_VOC, sampling_interval=1.0)
    nox_algo = GasIndexAlgorithm(ALGORITHM_TYPE_NOX, sampling_interval=1.0)

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

        # Кондиционирование (ТОЛЬКО для SGP41, строго до 10 секунд, нагреватель после НЕ выключаем!)
        if 1 == sensor.get_sensor_id():
            print("Conditioning SGP41 (10 s)...")
            for sec in range(10):
                t0 = ticks_ms()
                cond_raw = sensor.execute_conditioning()
                # Во время кондиционирования SGP41 уже отдает достаточно точный RAW_VOC,
                # поэтому включаю в работу VOC-фильтр
                voc_idx = voc_algo.get_gas_index(cond_raw.VOC)
                print(f"[{sec + 1}/10 s] Conditioning VOC raw: {cond_raw.VOC} -> VOC Index: {voc_idx}")
                elapsed = ticks_diff(ticks_ms(), t0)
                if elapsed < 1000:
                    sleep_ms(1000 - elapsed)
            print("Conditioning complete.")

        print("Starting continuous measurement (first 45s warm-up returns index 0)...")
        for _ in range(1000):
            t0 = ticks_ms()
            raw = sensor.measure_raw_signal()

            voc_idx = voc_algo.get_gas_index(raw.VOC)
            if raw.NOx is not None:
                nox_idx = nox_algo.get_gas_index(raw.NOx)
                print(f"VOC raw: {raw.VOC:5d} -> Index: {voc_idx:3d} | NOx raw: {raw.NOx:5d} -> Index: {nox_idx:3d}")
            else:
                print(f"VOC raw: {raw.VOC:5d} -> Index: {voc_idx:3d}")

            # Выдерживаем строгий период опроса 1000 мс
            elapsed = ticks_diff(ticks_ms(), t0)
            if elapsed < 1000:
                sleep_ms(1000 - elapsed)

    finally:
        # отключаю нагреватель при завершении
        sensor.turn_heater_off()
        print("Heater turned off.")
