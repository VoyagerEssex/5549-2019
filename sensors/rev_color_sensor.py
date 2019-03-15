import wpilib
import hal
from hal_impl import i2c_helpers, data
import typing

hal_data = data.hal_data

class ColorSensorBase(wpilib.SendableBase):

    def __init__(self):
        super().__init__()
        self.redChannel = 0
        self.blueChannel = 0
        self.greenChannel = 0

    def getColor(self, *args, **kwargs) -> int:

        raise NotImplementedError(
            "Implement 'getColor' in your class!"
        )

    def getRed(self, *args, **kwargs) -> int:

        raise NotImplementedError(
            "Implement 'getRed' in your class!"
        )

    def getBlue(self, *args, **kwargs) -> int:

        raise NotImplementedError(
            "Implement 'getBlue' in your class!"
        )

    def getGreen(self, *args, **kwargs) -> int:

        raise NotImplementedError(
            "Implement 'getGreen' in your class!"
        )

    def initSendable(self, builder: wpilib.SendableBuilder) -> None:
        builder.setSmartDashboardType("ColorSensor")
        builder.addDoubleProperty("Red", self.getRed, None)
        builder.addDoubleProperty("Blue", self.getBlue, None)
        builder.addDoubleProperty("Green", self.getGreen, None)

class REV_Color_Sim(i2c_helpers.I2CSimBase):

    def __init__(self, sensor):
        super().__init__()
        self.red = 0
        self.green = 0
        self.blue = 0
        self.clear = 0

    def initializeI2C(self, port, status):
        self.color_key = "rev_color_sensor_v2_%d_blue_color" % port

    def transactionI2C(
        self, port, deviceAddress, dataToSend, sendSize, dataReceived, receiveSize
    ):
        deviceAddress = 0x39
        sendSize = 0xFF
        receiveSize = 0xFF
        dataReceived[0] = 0xFF

        return 1

    def readI2C(self, port, deviceAddress, buffer, count):
        color = hal_data["robot"].get(self.color_key, 0)
        buffer[0] = (0xFF).to_bytes(1, "big")

        return 1

class REV_Color_Sensor_V2(ColorSensorBase):

    address = 0x39

    def __init__(self, port: wpilib.I2C.Port = 0):
        super().__init__()

        if port is None:
            port = wpilib.I2C.Port.kOnboard

        simPort = None
        if hal.HALIsSimulation():
            simPort = REV_Color_Sim(self)

        self.i2c = wpilib.I2C(port, self.address, simPort=simPort)
        self.clearChannel = 0
        self.setName("REV_Robotics_Color_Sensor_V2", port)

    def enable(self) -> None:
        self.i2c.write(0x03, 0xAB)
        self.i2c.write(0x01, 0xC0)  # Set ATIME to 64 cycles.
        self.i2c.write(0x0D, 0x02)  # Configure WLONG to influence WTIME
        self.i2c.write(0x03, 0xFF)  # Configure WTIME register.
        self.i2c.write(0x00, 0x0B)

    def getColor(self, addClear: bool = False) -> typing.List[int]:
        if addClear:
            self.clearChannel = self.readRawRegister(0x15)[0]
        self.readRegister(0x16)
        self.redChannel = int.from_bytes(self.readRawRegister(0x17), byteorder="big")
        self.readRegister(0x18)
        self.greenChannel = int.from_bytes(self.readRawRegister(0x19), byteorder="big")
        self.readRegister(0x1a)
        self.blueChannel = int.from_bytes(self.readRawRegister(0x1b), byteorder="big")

        color = [self.redChannel, self.greenChannel, self.blueChannel, self.clearChannel]
        return color

    def getRed(self):
        self.readRawRegister(0x16)
        redReg = int.from_bytes(self.readRawRegister(0x17), byteorder="big")
        return redReg

    def getGreen(self):
        self.readRawRegister(0x18)
        greenReg = int.from_bytes(self.readRawRegister(0x19), byteorder="big")
        return greenReg

    def getBlue(self):
        self.readRawRegister(0x1a)
        blueReg = int.from_bytes(self.readRawRegister(0x1b), byteorder="big")
        return blueReg

    def readRegister(self, register: int) -> int:
        return int.from_bytes(self.i2c.read(register, 1), byteorder="big")

    def readRawRegister(self, register: int) -> bytearray:
        return self.i2c.read(register, 1)