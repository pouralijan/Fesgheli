import threading
from time import sleep

from smbus2 import SMBus
import RPi.GPIO
from Logger import Logger


class I2C(threading.Thread):
    def __init__(self, smbus, address) -> None:
        super().__init__()
        self.logger = Logger(self.name)
        self._stop_event = threading.Event()
        self.smbus = SMBus(smbus)
        self.address = address
        self.value = None
        self._do_stop_event = False
        self.logger.info("Initialize I2C device ...")

    def start(self) -> None:
        super().start()
        sleep(0.09)

    def run(self) -> None:
        self.logger.info("Starting I2C device ...")
        while not self._stop_event.isSet():
            if self.value and self._do_stop_event:
                print("Stopping I2C device thread ...")
                self._stop_event.set()
            else:
                self._do_action()

    def stop(self):
        if self.value:
            print("Stopping I2C device thread ...")
            self._stop_event.set()
            del self
        else:
            self._do_stop_event = True

    def get_value(self):
        return self.value

    def change_address(self, new_address):
        pass

    def _do_action(self):
        pass


class SRF02(I2C):
    name = "SRF02"

    def _do_action(self):
        try:
            self.smbus.write_byte_data(self.address, 0x00, 0x51)
            sleep(0.08)
            high_value = self.smbus.read_byte_data(self.address, 0x02)
            low_value = self.smbus.read_byte_data(self.address, 0x03)
            self.value = (high_value << 8) + low_value
        except OSError as error:
            self.logger.error(error)
            exit()

    def change_address(self, new_address):
        command_register = 0x00
        change_command1 = 0xA0
        change_command2 = 0xAA
        change_command3 = 0xA5

        self.smbus.write_word_data(self.address, command_register, change_command1)
        sleep(0.24)
        self.smbus.write_word_data(self.address, command_register, change_command2)
        sleep(0.24)
        self.smbus.write_word_data(self.address, command_register, change_command3)
        sleep(0.24)
        self.smbus.write_word_data(self.address, command_register, new_address)
        print("Change {} I2C device address to {}.".format(self.address, new_address))


class CMPS03(I2C):
    name = "CMPS03"

    def __init__(self, smbus, address, calibration_pin=None) -> None:
        self.calibration_pin = calibration_pin
        super().__init__(smbus, address)
        if self.calibration_pin:
            RPi.GPIO.setmode(RPi.GPIO.BCM)
            RPi.GPIO.setup(self.calibration_pin, RPi.GPIO.IN)
            if not RPi.GPIO.input(self.calibration_pin):
                self.logger.error("Calibration is not completed.")
                exit()

    def _do_action(self):
        try:
            self.smbus.write_byte_data(self.address, 0x00, 0x51)
            # sleep(0.08)
            high_value = self.smbus.read_byte_data(self.address, 0x02)
            low_value = self.smbus.read_byte_data(self.address, 0x03)
            self.value = ((high_value << 8) + low_value) / 10
        except OSError as error:
            self.logger.error(error)
            exit(-1)

    def calibre(self):
        self.logger.info("Calibration is started ...")
        if self.calibration_pin:
            RPi.GPIO.setmode(RPi.GPIO.BCM)
            RPi.GPIO.setup(self.calibration_pin, RPi.GPIO.IN)
            self.logger.info("Calibration pin is: {}\n".format(RPi.GPIO.input(self.calibration_pin)))
        for i in ["North", "East", "South", "West"]:
            if int(input("\nSet {} insert 1 to OK: ".format(i))) == 1:
                self.smbus.write_byte_data(self.address, 15, 0xFF)
                sleep(1)
            if self.calibration_pin:
                self.logger.info("Calibration pin is: {}\n".format(RPi.GPIO.input(self.calibration_pin)))
        if self.calibration_pin:
            self.logger.info("Calibration pin is: {}\n".format(RPi.GPIO.input(self.calibration_pin)))
