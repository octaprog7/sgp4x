# micropython
# MIT license

# Please read this before use!: https://www.sciosense.com/products/environmental-sensors/digital-multi-gas-sensor/
from machine import I2C, SoftI2C, Pin
import sgp4Xmod
from sensor_pack_2.bus_service import I2cAdapter
from time import sleep_ms


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(id=1, scl=Pin(27), sda=Pin(26), freq=400_000)  # on Arduino Nano RP2040 Connect and Pico W tested!
    # i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)
    sda_pin = 8  # Replace with your SDA pin number
    scl_pin = 9  # Replace with your SCL pin number
    # ESP32-C3 не имеет аппаратного I2C. Там, где он есть, используйте I2C вместо SoftI2C!
    bus = SoftI2C(sda=Pin(sda_pin), scl=Pin(scl_pin), freq=400_000)
    adaptor = I2cAdapter(bus)
    sensor = sgp4Xmod.SGP4X(adapter=adaptor, address=0x59, check_crc=True, sensor_id = 0)
    # 3.6 Поведение датчика при запуске
    # Датчик начинает включаться после достижения порогового напряжения включения 1,7 В и переходит в режим ожидания максимум через 0,6 мс.
    # В этом состоянии датчик готов к приему команд от ведущего устройства!
    sleep_ms(600)
    _raw = None
    try:
        sen_id = sensor.get_id()
        print(f"{sen_id}")
        test_result = sensor.execute_self_test()
        print(f"test result: 0x{test_result:x}")
        if 0xD400 == test_result:
            print("Все тесты пройдены успешно!")
        else:
            print("Один или несколько тестов завершились с ошибкой!")
        sleep_ms(100)
        try:
            # Если вы перегреете датчик, он сломается навсегда! 10 секунд максимум (см. документацию!)
            for _ in range(3):
                if 0 == sensor.get_sensor_id():
                    break # у SGP40 нет такой команды!
                _raw = sensor.execute_conditioning()
                print(f"conditioning return: {_raw}")
                sleep_ms(1000)
        finally:
            sensor.turn_heater_off()
        
        for _ in range(1000):
            _raw = sensor.measure_raw_signal()
            print(f"{_raw}")
            sleep_ms(1000)
    finally:
        sensor.turn_heater_off()

