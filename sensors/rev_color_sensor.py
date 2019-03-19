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
        self.color_key = "rev_color_sensor_v2_%d_color" % port

    def transactionI2C(
        self, port, deviceAddress, dataToSend, sendSize, dataReceived, receiveSize
    ):
        deviceAddress = 0x39
        sendSize = 0xFF
        receiveSize = 0xFF
        dataReceived[0] = 0xFF
        port = wpilib.I2C.Port.kOnboard

        return 1

    def readI2C(self, port, deviceAddress, buffer, count):
        color = hal_data["robot"].get(self.color_key, 0)
        if count is 2:
            buffer[1] = (0xFF).to_bytes(1, "big")
            buffer[0] = (0xFF).to_bytes(1, "big")
        elif count is 1:
            buffer[0] = (0xFF).to_bytes(1, "big")

        return count

class REV_Color_Sensor_V2(ColorSensorBase):

    ADDRESS = 0x39

    def __init__(self, port: wpilib.I2C.Port = 0):
        super().__init__()

        if port is None:
            port = wpilib.I2C.Port.kOnboard

        simPort = None
        if hal.HALIsSimulation():
            simPort = REV_Color_Sim(self)

        self.i2c = wpilib.I2C(port, self.ADDRESS, simPort=simPort)
        self.clearChannel = 0
        self.setName("REV_Robotics_Color_Sensor_V2", port)

    def enable(self) -> None:
        self.i2c.write(0x00, 0x83)

    def getColor(self, addClear: bool = False) -> typing.List[int]:
        if addClear:
            self.clearChannel = self.readRawRegister(0x80 | 0x20 | 0x14)[0]
        self.redChannel = int.from_bytes(self.readRawRegister(0x80 | 0x20 | 0x16), byteorder="big")
        self.greenChannel = int.from_bytes(self.readRawRegister(0x80 | 0x20 | 0x18), byteorder="big")
        self.blueChannel = int.from_bytes(self.readRawRegister(0x80| 0x20| 0x1a), byteorder="big")
        
        color = [None] * 3
        if addClear:
            color = [self.redChannel, self.greenChannel, self.blueChannel]
            color.append(self.clearChannel)
        else:
            color = [self.redChannel, self.greenChannel, self.blueChannel, self.clearChannel]
        return color

    def getRed(self):
        redRegLow = int.from_bytes(self.readRawRegister(0x80 | 0x20 | 0x16, 2), byteorder="big")
        # redRegHigh = int.from_bytes(self.readRawRegister(0x17, 1), byteorder="big")
        return redRegLow

    def getGreen(self):
        greenRegLow = int.from_bytes(self.readRawRegister(0x80 | 0x20 | 0x18, 2), byteorder="big")
        # greenRegHigh = int.from_bytes(self.readRawRegister(0x19, 1), byteorder="big")
        return greenRegLow

    def getBlue(self):
        blueRegLow = int.from_bytes(self.readRawRegister(0x80 | 0x20 | 0x1a, 2), byteorder="big")
        # blueRegHigh = int.from_bytes(self.readRawRegister(0x1b, 1), byteorder="big")
        return blueRegLow

    def readRegister(self, register: int, bytes: int) -> int:
        return int.from_bytes(self.i2c.read(register, bytes), byteorder="big")

    def readRawRegister(self, register: int, bytes: int) -> bytearray:
        return self.i2c.read(register, bytes)
