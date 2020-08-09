#!/usr/bin/env python3

import RPi.GPIO
from Logger import Logger
import time

class DCMotorDriver(object):
    def __del__(self):
        if self.pwm:
            self.pwm.stop()
        RPi.GPIO.cleanup()

    def __init__(self, motor_pin_0, motor_pin_1, pwm=None,
                 gpio_mod=RPi.GPIO.BCM, frequency=50) -> None:
        RPi.GPIO.setmode(gpio_mod)
        self.motor_pin_0 = motor_pin_0
        self.motor_pin_1 = motor_pin_1
        RPi.GPIO.setup(self.motor_pin_0, RPi.GPIO.OUT)
        RPi.GPIO.setup(self.motor_pin_1, RPi.GPIO.OUT)
        self.pwm = None
        if pwm:
            RPi.GPIO.setup(pwm, RPi.GPIO.OUT)
            self.pwm = RPi.GPIO.PWM(pwm, frequency)

    def left(self, pwm=100.0):
        RPi.GPIO.output(self.motor_pin_0, RPi.GPIO.HIGH)
        RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.LOW)
        if self.pwm:
            self.pwm.start(pwm)

    def right(self, pwm=100.0):
        RPi.GPIO.output(self.motor_pin_0, RPi.GPIO.LOW)
        RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.HIGH)
        if self.pwm:
            self.pwm.start(pwm)

    def stop(self):
        RPi.GPIO.output(self.motor_pin_0, RPi.GPIO.LOW)
        RPi.GPIO.output(self.motor_pin_1, RPi.GPIO.LOW)


class MotorsCountError(Exception):
    def __init__(self, *args: object, **kwargs: object) -> None:
        self.logger = Logger("MotorsCountError")
        for arg in args:
            self.logger.error(arg)


class BaseMovementUnit(object):
    MOTORS = None
    logger = Logger("MovementUnit")

    def forward(self, pwm: int = None) -> None:
        for motor in self.MOTORS:
            motor.right(pwm)

    def backward(self, pwm: int = None) -> None:
        for motor in self.MOTORS:
            motor.left(pwm)

    def rotate(self, degree: int, pwm: int = None) -> None:
        pass

    def stop(self) -> None:
        for motor in self.MOTORS:
            motor.stop()


class MovementUnit(BaseMovementUnit):
    def __init__(self, *args, **kwargs) -> None:
        self.movement_unit = BaseMovementUnit()
        try:
            if 2 <= len(args) < 5:
                self.logger.info("Initialize motors ...")
                if len(args) is 2:
                    self.movement_unit = TwoMotors(args, kwargs)
                elif len(args) is 3:
                    self.movement_unit = ThreeMotors(args, kwargs)
                elif len(args) is 4:
                    self.movement_unit = FourMotors(args, kwargs)
                else:
                    raise MotorsCountError("This version of MomentUnit support motor count between 2 and 4")
            else:
                raise MotorsCountError("This version of MomentUnit support motor count between 2 and 4")
        except MotorsCountError:
            exit(-1)

    def forward(self, pwm: int = None) -> None:
        self.movement_unit.forward(pwm)

    def backward(self, pwm: int = None) -> None:
        self.movement_unit.backward(pwm)

    def rotate(self, degree: int, pwm: int = None) -> None:
        self.movement_unit.rotate(degree, pwm)

    def stop(self) -> None:
        self.movement_unit.stop()


class TwoMotors(BaseMovementUnit):
    def __init__(self, motors, kwargs) -> None:
        self.compass = None
        if "compass" in kwargs:
            self.compass = kwargs["compass"]
        self.logger.info("Tow Motors")
        self.MOTORS = motors
        self.right_motor = motors[0]
        self.left_motor = motors[1]

    def forward(self, pwm: int = None) -> None:
        super(TwoMotors, self).forward(pwm)

    def backward(self, pwm: int = None) -> None:
        self.logger.info("Tow Motors, Backward")
        super(TwoMotors, self).backward(pwm)

    def rotate(self, degree: int, pwm: int = None) -> None:
        self.logger.info("Tow Motors, Rotate with {} degree.".format(degree))
        if self.compass:
            self._rotate(degree, pwm)
        else:
            if degree < 0:
                self.left_motor.right(pwm)
                self.right_motor.left(pwm)
                time.sleep(abs(degree))
            else:
                self.left_motor.left(pwm)
                self.right_motor.right(pwm)
                time.sleep(abs(degree))
            self.stop()

    def _rotate(self, degree: int, pwm: int = None) -> None:
        self.compass.start()
        start_degree = self.compass.get_value()
        current_degree = start_degree
        destination_degree = (degree + start_degree)
        if destination_degree > 360:
            destination_degree -= 360
        if destination_degree < 0:
            destination_degree += 360

        destination_degree_list = [i for i in
                                   range(int(destination_degree) - 5, int(destination_degree) + 5)]
        if degree < 0:
            while int(current_degree) not in destination_degree_list:
                current_degree = self.compass.get_value()
                print(current_degree, destination_degree_list)
                self.left_motor.right(pwm)
                self.right_motor.left(pwm)

        else:
            while int(current_degree) not in destination_degree_list:
                current_degree = self.compass.get_value()
                print(current_degree, destination_degree_list)
                self.left_motor.left(pwm)
                self.right_motor.right(pwm)

        self.left_motor.stop()
        self.right_motor.stop()

        self.compass.stop()


class ThreeMotors(BaseMovementUnit):
    def __init__(self, motors, kwargs) -> None:
        print("Three Motors")

    def forward(self, pwm: int = None) -> None:
        print("Three Motors, Forward")

    def backward(self, pwm: int = None) -> None:
        print("Three Motors, Backward")

    def rotate(self, degree: int, pwm: int = None) -> None:
        print("Three Motors, Rotate", degree)


class FourMotors(BaseMovementUnit):
    def __init__(self, motors, kwargs) -> None:
        print("Four Motors")

    def forward(self, pwm: int = None) -> None:
        print("Four Motors, Forward")

    def backward(self, pwm: int = None) -> None:
        print("Four Motors, Backward")

    def rotate(self, degree: int, pwm: int = None) -> None:
        print("Four Motors, Rotate", degree)
