'''
Destination: Deep Space 2019 - GEMINI from Gryphon Robotics
'''

import wpilib
import logging
from math import *
from wpilib.drive import DifferentialDrive
from networktables import NetworkTables
from ctre import *
from sensors import REV_Color_Sensor_V2


class Gemini(wpilib.TimedRobot):
    ''' values for navx'''

    def robotInit(self):
        ''' Initialization of robot objects. '''

        ''' Talon SRX Initialization '''
        # drive train motors
        self.frontRightMotor = WPI_TalonSRX(4)
        self.rearRightMotor = WPI_TalonSRX(3)
        self.frontLeftMotor = WPI_TalonSRX(1)
        self.rearLeftMotor = WPI_TalonSRX(2)

        ''' Encoders '''
        # drive train encoders
        self.rightEncoder = self.frontRightMotor
        self.leftEncoder = self.frontLeftMotor

        # lift encoder
        self.liftEncoder = wpilib.Encoder(8, 9)

        # liftArm encoder
        self.liftArmEncoder = wpilib.Encoder(5, 6)

        ''' Motor Groups '''
        # drive train motor groups
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # drive train drive group
        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        ''' Victor SPX Initialization '''
        # lift motors
        self.liftOne = WPI_VictorSPX(1)
        self.liftTwo = WPI_VictorSPX(2)
        self.lift = wpilib.SpeedControllerGroup(self.liftOne, self.liftTwo)

        # lift arm motors
        self.liftArmOne = WPI_VictorSPX(3)
        self.liftArmTwo = WPI_VictorSPX(4)
        self.liftArm = wpilib.SpeedControllerGroup(self.liftArmOne, self.liftArmTwo)

        # cargo intake motor
        self.cargo = WPI_VictorSPX(5)

        ''' Controller Initialization '''
        # joystick - 0, 1 | controller - 2
        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.xbox = wpilib.Joystick(2)
        self.buttonBox = wpilib.Joystick(3)

        ''' Button Status'''
        self.buttonStatus = [False, False, False, False, False, False, False]

        ''' Pneumatic Initialization '''
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoidOne = wpilib.DoubleSolenoid(0, 1)  # gear shifting
        self.DoubleSolenoidTwo = wpilib.DoubleSolenoid(2, 3)  # hatch panel claw
        self.DoubleSolenoidThree = wpilib.DoubleSolenoid(4, 5)  # hatch panel ejection
        self.Compressor.start()

        ''' Smart Dashboard '''
        # connection for logging & Smart Dashboard
        logging.basicConfig(level=logging.DEBUG)
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.55.49.2')
        self.sd.putString("  ", "Connection")

        # Smart Dashboard classes
        self.PDP = wpilib.PowerDistributionPanel()
        self.roboController = wpilib.RobotController()
        self.DS = wpilib.DriverStation.getInstance()

        ''' Sensors '''
        # Hall Effect Sensor
        self.Hall = wpilib.DigitalInput(7)
        self.ultrasonic = wpilib.AnalogInput(2)
        self.cargoUltrasonic = wpilib.AnalogInput(3)

        ''' Timer '''
        self.timer = wpilib.Timer()

        ''' Camera '''
        # initialization of the HTTP camera
        wpilib.CameraServer.launch('vision.py:main')
        self.sd.putString("", "Top Camera")
        self.sd.putString(" ", "Bottom Camera")

        # Initialization and configuration of I2C interface with color sensor.
        self.colorSensor = REV_Color_Sensor_V2(wpilib.I2C.Port.kOnboard)


    def autonomousInit(self):
        ''' Executed each time the robot enters autonomous. '''

        # timer config
        self.timer.reset()
        self.timer.start()

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        self.liftEncoder.reset()

    def autonomousPeriodic(self):
        ''' Called periodically during autonomous. '''

        '''Test Methods'''

        def encoder_test():
            ''' Drives robot set encoder distance away '''
            self.rightPos = fabs(self.rightEncoder.getQuadraturePosition())
            self.leftPos = fabs(self.leftEncoder.getQuadraturePosition())
            self.distIn = (((self.leftPos + self.rightPos) / 2) / 4096) * 18.84955
            if 0 <= self.distIn <= 72:
                self.drive.tankDrive(0.5, 0.5)
            else:
                self.drive.tankDrive(0, 0)

        def Diagnostics():
            ''' Smart Dashboard Tests'''
            self.sd.putNumber("Temperature: ", self.PDP.getTemperature())
            self.sd.putNumber("Battery Voltage: ", self.roboController.getBatteryVoltage())
            self.sd.putBoolean(" Browned Out?", self.roboController.isBrownedOut)

            # Smart Dashboard diagnostics
            self.sd.putNumber("Right Encoder Speed: ", abs(self.rightEncoder.getQuadratureVelocity()))
            self.sd.putNumber("Left Encoder Speed: ", abs(self.leftEncoder.getQuadratureVelocity()))
            self.sd.putNumber("Lift Encoder: ", self.liftEncoder.getDistance())

        def Pressure():
            self.Compressor.start()

        def cargoOne():
            if self.liftEncoder.get() <= 133:  # Cargo 1
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 133:
                self.lift.set(0.05)
                self.buttonStatus[4] = False

        def cargoTwo():
            if self.liftEncoder.get() <= 270:  # Cargo 2
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 270:
                self.lift.set(0.05)
                self.buttonStatus[2] = False

        def cargoThree():
            if self.liftEncoder.get() <= 415:  # Cargo 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 415:
                self.lift.set(0.05)
                self.buttonStatus[0] = False

        def hatchOne():
            if self.liftEncoder.get() <= 96:  # Hatch 1
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 96:
                self.lift.set(0.05)
                self.buttonStatus[5] = False

        def hatchTwo():
            if self.liftEncoder.get() <= 237:  # Hatch 2
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 237:
                self.lift.set(0.05)
                self.buttonStatus[3] = False

        def hatchThree():
            if self.liftEncoder.get() <= 378:  # Hatch 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 378:
                self.lift.set(0.05)
                self.buttonStatus[1] = False

        def liftEncoderReset():
            self.lift.set(0.01)
            if self.Hall.get() is True:
                self.liftEncoder.reset()

        ''' Button Status Toggle '''
        if self.buttonBox.getRawButtonPressed(1):
            self.buttonStatus[0] = not self.buttonStatus[0]
            if True in self.buttonStatus[1:]:
                self.buttonStatus[1:] = [False for aa in self.buttonStatus[1:]]
        elif self.buttonBox.getRawButtonPressed(2):
            self.buttonStatus[1] = not self.buttonStatus[1]
            if True in self.buttonStatus[0 and 2:]:
                self.buttonStatus[0 and 2:] = [False for aa in self.buttonStatus[0 and 2:]]
        elif self.buttonBox.getRawButtonPressed(3):
            self.buttonStatus[2] = not self.buttonStatus[2]
            if True in self.buttonStatus[0:1 and 3:]:
                self.buttonStatus[0:1 and 3:] = [False for aa in self.buttonStatus[0:1 and 3:]]
        elif self.buttonBox.getRawButtonPressed(4):
            self.buttonStatus[3] = not self.buttonStatus[3]
            if True in self.buttonStatus[0:2 and 4:]:
                self.buttonStatus[0:2 and 4:] = [False for aa in self.buttonStatus[0:2 and 4:]]
        elif self.buttonBox.getRawButtonPressed(5):
            self.buttonStatus[4] = not self.buttonStatus[4]
            if True in self.buttonStatus[0:3 and 5:]:
                self.buttonStatus[0:3 and 5:] = [False for aa in self.buttonStatus[0:3 and 5:]]
        elif self.buttonBox.getRawButtonPressed(6):
            self.buttonStatus[5] = not self.buttonStatus[5]
            if True in self.buttonStatus[0:4 and 6]:
                self.buttonStatus[0:4 and 6] = [False for aa in self.buttonStatus[0:4 and 6]]
        elif self.buttonBox.getRawButtonPressed(7):
            self.buttonStatus[6] = not self.buttonStatus[6]
            if True in self.buttonStatus[0:5]:
                self.buttonStatus[0:5] = [False for aa in self.buttonStatus[0:5]]

        ''' Button Box Level Mapping '''
        if self.buttonStatus[0] is True:
            cargoThree()
        elif self.buttonStatus[1] is True:
            hatchThree()
        elif self.buttonStatus[2] is True:
            cargoTwo()
        elif self.buttonStatus[3] is True:
            hatchTwo()
        elif self.buttonStatus[4] is True:
            cargoOne()
        elif self.buttonStatus[5] is True:
            hatchOne()
        elif self.buttonStatus[6] is True:
            liftEncoderReset()

        ''' Test Execution '''
        if self.DS.getGameSpecificMessage() == "pressure":
            Pressure()
        elif self.DS.getGameSpecificMessage() == "diagnostics":
            Diagnostics()

        ''' Smart Dashboard '''
        # compressor state
        if self.Compressor.enabled() is True:
            self.sd.putString("Compressor Status: ", "Enabled")
        elif self.Compressor.enabled() is False:
            self.sd.putString("Compressor Status: ", "Disabled")

        # gear state
        if self.DoubleSolenoidOne.get() == 1:
            self.sd.putString("Gear Shift: ", "High Speed")
        elif self.DoubleSolenoidOne.get() == 2:
            self.sd.putString("Gear Shift: ", "Low Speed")

        # ejector state
        if self.DoubleSolenoidThree.get() == 2:
            self.sd.putString("Ejector Pins: ", "Ejected")
        elif self.DoubleSolenoidThree.get() == 1:
            self.sd.putString("Ejector Pins: ", "Retracted")

        # claw state
        if self.DoubleSolenoidTwo.get() == 2:
            self.sd.putString("Claw: ", "Open")
        elif self.DoubleSolenoidTwo.get() == 1:
            self.sd.putString("Claw: ", "Closed")

        ''' Ultrasonic stuff '''
        # robot ultrasonic
        self.ultraValue = self.ultrasonic.getVoltage()
        if 0.142 <= self.ultraValue <= 0.146:
            self.sd.putString("PLAYER STATION RANGE: ", "YES!!!!")
        else:
            self.sd.putString("PLAYER STATION RANGE: ", "NO!")

        # self.sd.putNumber("Ultrasonic Voltage: ", self.ultraValue)

        # cargo ultrasonic
        self.cargoUltraValue = self.cargoUltrasonic.getVoltage()

        if 0.70 <= self.cargoUltraValue <= 1.56:
            self.sd.putString("HATCH RANGE: ", "HATCH IN RANGE")
        else:
            self.sd.putString("HATCH RANGE: ", "NOT IN RANGE")

        ''' Pneumatics Control '''
        # compressor
        if self.xbox.getRawButton(9):
            self.Compressor.stop()
        elif self.xbox.getRawButton(10):
            self.Compressor.start()
        elif self.rightStick.getRawButton(1):  # shift right
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.leftStick.getRawButton(1):  # shift left
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(3):  # open claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(2):  # close claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(4):  # eject
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(1):  # retract
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)

        ''' Victor SPX (Lift, Lift Arm, Cargo) '''
        # lift control
        if True in self.buttonStatus:
            if self.xbox.getRawButton(5):  # hold
                self.lift.set(0.05)
            elif self.xbox.getRawAxis(3):  # up
                self.lift.set(self.xbox.getRawAxis(3) / 1.5)
            elif self.xbox.getRawAxis(2):  # down
                self.lift.set(-self.xbox.getRawAxis(2) * 0.25)
            else:
                self.lift.set(0)

        # four-bar control
        if self.xbox.getRawButton(6):
            self.liftArm.set(0.05)
        elif not self.xbox.getRawButton(6):
            self.liftArm.set(-self.xbox.getRawAxis(1) / 4.0)
        else:
            self.liftArm.set(0)

        # cargo intake control
        if self.xbox.getRawButton(7):
            self.cargo.set(0.12)
        elif self.xbox.getRawAxis(5):  # take in
            self.cargo.set(self.xbox.getRawAxis(5) * 0.75)

        # controller mapping for tank steering
        rightAxis = self.rightStick.getRawAxis(1)
        leftAxis = self.leftStick.getRawAxis(1)

        # drives drive system using tank steering
        if self.DoubleSolenoidOne.get() == 1:  # if on high gear
            self.divisor = 1.2  # 90% of high speed
        elif self.DoubleSolenoidOne.get() == 2:  # if on low gear
            self.divisor = 1.2  # normal slow speed
        else:
            self.divisor = 1.0

        if leftAxis != 0:
            self.leftSign = leftAxis / fabs(leftAxis)
        else:
            self.leftSign = 0
        if rightAxis != 0:
            self.rightSign = rightAxis / fabs(rightAxis)
        else:
            self.rightSign = 0

        self.drive.tankDrive(-(self.leftSign) * (1 / self.divisor) * (leftAxis ** 2),
                             -(self.rightSign) * (1 / self.divisor) * (rightAxis ** 2))

        # self.drive.tankDrive(-leftAxis / self.divisor, -rightAxis/ self.divisor)  # drive divided by appropriate divisor

    def teleopInit(self):
        ''' Executed at the start of teleop mode. '''

        self.drive.setSafetyEnabled(True)

        # drive train encoder reset
        self.rightEncoder.setQuadraturePosition(0, 0)
        self.leftEncoder.setQuadraturePosition(0, 0)

        # lift encoder rest
        self.liftEncoder.reset()

        # compressor
        self.Compressor.start()

    def teleopPeriodic(self):
        ''' Periodically executes methods during the teleop mode. '''
        '''        
        self.sd.putString(" ", "Match Info")
        self.sd.putString("Event Name: ", self.DS.getEventName())
        self.sd.putNumber("Match Time: ", self.timer.getMatchTime())
        self.sd.putNumber("Match Number: ", self.DS.getMatchTime())
        self.sd.putNumber("Location: ", self.DS.getLocation())
        if self.DS.getMatchType() == 3:
            self.sd.putString("Match Type: ", "Elimination")
        elif self.DS.getMatchType() == 1:
            self.sd.putString("Match Type: ", "Practice")
        elif self.DS.getMatchType() == 2:
            self.sd.putString("Match Type: ", "Qualification")
        else:
            self.sd.putString("Match Type: ", "None")

        if self.DS.getAlliance() == 0:
            self.sd.putString("Alliance: ", "Red")
        elif self.DS.getAlliance() == 1:
            self.sd.putString("Alliance: ", "Blue")
        else:
            self.sd.putString("Alliance: ", "Invalid")
        '''

        def cargoOne():
            if self.liftEncoder.get() <= 133:  # Cargo 1
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 133:
                self.lift.set(0.05)
                self.buttonStatus[4] = False

        def cargoTwo():
            if self.liftEncoder.get() <= 270:  # Cargo 2
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 270:
                self.lift.set(0.05)
                self.buttonStatus[2] = False

        def cargoThree():
            if self.liftEncoder.get() <= 415:  # Cargo 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 415:
                self.lift.set(0.05)
                self.buttonStatus[0] = False

        def hatchOne():
            if self.liftEncoder.get() <= 96:  # Hatch 1
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 96:
                self.lift.set(0.05)
                self.buttonStatus[5] = False

        def hatchTwo():
            if self.liftEncoder.get() <= 237:  # Hatch 2
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 237:
                self.lift.set(0.05)
                self.buttonStatus[3] = False

        def hatchThree():
            if self.liftEncoder.get() <= 378:  # Hatch 3
                self.lift.set(0.5)
            elif self.liftEncoder.get() > 378:
                self.lift.set(0.05)
                self.buttonStatus[1] = False

        def liftEncoderReset():
            self.lift.set(0.01)
            if self.Hall.get() is True:
                self.liftEncoder.reset()

        ''' Button Status Toggle '''
        if self.buttonBox.getRawButtonPressed(1):
            self.buttonStatus[0] = not self.buttonStatus[0]
            if True in self.buttonStatus[1:]:
                self.buttonStatus[1:] = [False for aa in self.buttonStatus[1:]]
        elif self.buttonBox.getRawButtonPressed(2):
            self.buttonStatus[1] = not self.buttonStatus[1]
            if True in self.buttonStatus[0 and 2:]:
                self.buttonStatus[0 and 2:] = [False for aa in self.buttonStatus[0 and 2:]]
        elif self.buttonBox.getRawButtonPressed(3):
            self.buttonStatus[2] = not self.buttonStatus[2]
            if True in self.buttonStatus[0:1 and 3:]:
                self.buttonStatus[0:1 and 3:] = [False for aa in self.buttonStatus[0:1 and 3:]]
        elif self.buttonBox.getRawButtonPressed(4):
            self.buttonStatus[3] = not self.buttonStatus[3]
            if True in self.buttonStatus[0:2 and 4:]:
                self.buttonStatus[0:2 and 4:] = [False for aa in self.buttonStatus[0:2 and 4:]]
        elif self.buttonBox.getRawButtonPressed(5):
            self.buttonStatus[4] = not self.buttonStatus[4]
            if True in self.buttonStatus[0:3 and 5:]:
                self.buttonStatus[0:3 and 5:] = [False for aa in self.buttonStatus[0:3 and 5:]]
        elif self.buttonBox.getRawButtonPressed(6):
            self.buttonStatus[5] = not self.buttonStatus[5]
            if True in self.buttonStatus[0:4 and 6]:
                self.buttonStatus[0:4 and 6] = [False for aa in self.buttonStatus[0:4 and 6]]
        elif self.buttonBox.getRawButtonPressed(7):
            self.buttonStatus[6] = not self.buttonStatus[6]
            if True in self.buttonStatus[0:5]:
                self.buttonStatus[0:5] = [False for aa in self.buttonStatus[0:5]]

        ''' Button Box Level Mapping '''
        if self.buttonStatus[0] is True:
            cargoThree()
        elif self.buttonStatus[1] is True:
            hatchThree()
        elif self.buttonStatus[2] is True:
            cargoTwo()
        elif self.buttonStatus[3] is True:
            hatchTwo()
        elif self.buttonStatus[4] is True:
            cargoOne()
        elif self.buttonStatus[5] is True:
            hatchOne()
        elif self.buttonStatus[6] is True:
            liftEncoderReset()

        ''' Smart Dashboard '''
        # compressor state
        if self.Compressor.enabled() is True:
            self.sd.putString("Compressor Status: ", "Enabled")
        elif self.Compressor.enabled() is False:
            self.sd.putString("Compressor Status: ", "Disabled")

        # gear state
        if self.DoubleSolenoidOne.get() == 1:
            self.sd.putString("Gear Shift: ", "High Speed")
        elif self.DoubleSolenoidOne.get() == 2:
            self.sd.putString("Gear Shift: ", "Low Speed")

        # ejector state
        if self.DoubleSolenoidThree.get() == 2:
            self.sd.putString("Ejector Pins: ", "Ejected")
        elif self.DoubleSolenoidThree.get() == 1:
            self.sd.putString("Ejector Pins: ", "Retracted")

        # claw state
        if self.DoubleSolenoidTwo.get() == 2:
            self.sd.putString("Claw: ", "Open")
        elif self.DoubleSolenoidTwo.get() == 1:
            self.sd.putString("Claw: ", "Closed")

        ''' Ultrasonic '''
        self.ultraValue = self.ultrasonic.getVoltage()

        if 0.142 <= self.ultraValue <= 0.146:
            self.sd.putString("PLAYER STATION RANGE: ", "YES!!!!")
        else:
            self.sd.putString("PLAYER STATION RANGE: ", "NO!")

        # self.sd.putNumber("Ultrasonic Voltage: ", self.ultraValue)

        # cargo ultrasonic
        self.cargoUltraValue = self.cargoUltrasonic.getVoltage()

        if 0.70 <= self.cargoUltraValue <= 1.56:
            self.sd.putString("HATCH RANGE: ", "HATCH IN RANGE")
        else:
            self.sd.putString("HATCH RANGE: ", "NOT IN RANGE")

        # # button states
        # self.sd.putBoolean("Button 1 (Cargo 3): ", self.buttonStatusOne)
        # self.sd.putBoolean("Button 2 (Hatch 3): ", self.buttonStatusTwo)
        # self.sd.putBoolean("Button 3 (Cargo 2): ", self.buttonStatusThree)
        # self.sd.putBoolean("Button 4 (Hatch 2): ", self.buttonStatusFour)
        # self.sd.putBoolean("Button 5 (Cargo 1): ", self.buttonStatusFive)
        # self.sd.putBoolean("Button 6 (Hatch 1): ", self.buttonStatusSix)
        # self.sd.putBoolean("Button 7 (Reset): ", self.buttonStatusSeven)

        ''' Pneumatics Control '''
        # compressor
        if self.xbox.getRawButton(9):
            self.Compressor.stop()
        elif self.xbox.getRawButton(10):
            self.Compressor.start()
        elif self.rightStick.getRawButton(1):  # shift right
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.leftStick.getRawButton(1):  # shift left
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(3):  # open claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(2):  # close claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif self.xbox.getRawButton(4):  # eject
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)
        elif self.xbox.getRawButton(1):  # retract
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)

        ''' Victor SPX (Lift, Lift Arm, Cargo) '''
        # lift control
        if True in self.buttonStatus:
            if self.xbox.getRawAxis(3):  # up
                self.lift.set(self.xbox.getRawAxis(3) / 1.5)
            elif self.xbox.getRawAxis(2):  # down
                self.lift.set(-self.xbox.getRawAxis(2) * 0.25)
            elif self.xbox.getRawButton(5):  # hold
                self.lift.set(0.05)
            else:
                self.lift.set(0)

        # four-bar control
        if self.xbox.getRawButton(6):
            self.liftArm.set(0.05)
        elif not self.xbox.getRawButton(6):
            self.liftArm.set(-self.xbox.getRawAxis(1) / 4.0)
        else:
            self.liftArm.set(0)

        # cargo intake control
        if self.xbox.getRawButton(7):
            self.cargo.set(0.12)
        elif self.xbox.getRawAxis(5):  # take in
            self.cargo.set(self.xbox.getRawAxis(5) * 0.75)

        # controller mapping for tank steering
        rightAxis = self.rightStick.getRawAxis(1)
        leftAxis = self.leftStick.getRawAxis(1)

        # drives drive system using tank steering
        if self.DoubleSolenoidOne.get() == 1:  # if on high gear
            self.divisor = 1.2  # 90% of high speed
        elif self.DoubleSolenoidOne.get() == 2:  # if on low gear
            self.divisor = 1.2  # normal slow speed
        else:
            self.divisor = 1.0

        if leftAxis != 0:
            self.leftSign = leftAxis / fabs(leftAxis)
        else:
            self.leftSign = 0
        if rightAxis != 0:
            self.rightSign = rightAxis / fabs(rightAxis)
        else:
            self.rightSign = 0

        self.drive.tankDrive(-(self.leftSign) * (1 / self.divisor) * (leftAxis ** 2),
                             -(self.rightSign) * (1 / self.divisor) * (rightAxis ** 2))
        # self.drive.tankDrive(-leftAxis / self.divisor, -rightAxis/ self.divisor)  # drive divided by appropriate divisor

if __name__ == '__main__':
    wpilib.run(Gemini)
