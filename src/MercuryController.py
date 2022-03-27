import serial


class MercuryController:
    def __init__(self, port, baud):
        self.serial = serial.Serial(port, baud)
        self.serial.flushInput()

    def led_on(self, pin):
        self.serial.write("LED {} on".format(pin))

    def led_off(self, pin):
        self.serial.write("LED {} off".format(pin))

    def set_servo_percent(self, servo, percent):
        self.serial.write("servo {0} {1}".format(servo, percent))
