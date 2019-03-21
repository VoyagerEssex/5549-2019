'''
5549 presents:
Destination: Deep Space 2019 - GEMINI from Gryphon Robotics

 by:
 _    _  _____  __   __ _______  ______ _______  ______
  \  /  |     |   \_/   |_____| |  ____ |______ |_____/
   \/   |_____|    |    |     | |_____| |______ |    \_

 .. _______  _____  __   _ __   _  _____   ______ ..
 '' |       |     | | \  | | \  | |     | |_____/ ''
    |_____  |_____| |  \_| |  \_| |_____| |    \_

 _______ _______ _______ _______ _     _
 |______ |______ |______ |______  \___/
 |______ ______| ______| |______ _/   \_

---------------------------------------------------------
  _____  _______  ______ _______ _____ _______
 |_____] |_____| |_____/ |______   |   |_____|
 |       |     | |    \_ ______| __|__ |     |

Leading programming affiliate members of:

      ___           ___                       ___         ___           ___           ___                     
     /\__\         /\  \                     /\  \       /\  \         /\  \         /\  \                    
    /:/ _/_       /::\  \         ___       /::\  \      \:\  \       /::\  \        \:\  \                   
   /:/ /\  \     /:/\:\__\       /|  |     /:/\:\__\      \:\  \     /:/\:\  \        \:\  \                  
  /:/ /::\  \   /:/ /:/  /      |:|  |    /:/ /:/  /  ___ /::\  \   /:/  \:\  \   _____\:\  \                 
 /:/__\/\:\__\ /:/_/:/__/___    |:|  |   /:/_/:/  /  /\  /:/\:\__\ /:/__/ \:\__\ /::::::::\__\                
 \:\  \ /:/  / \:\/:::::/  /  __|:|__|   \:\/:/  /   \:\/:/  \/__/ \:\  \ /:/  / \:\~~\~~\/__/                
  \:\  /:/  /   \::/~~/~~~~  /::::\  \    \::/__/     \::/__/       \:\  /:/  /   \:\  \                      
   \:\/:/  /     \:\~~\      ~~~~\:\  \    \:\  \      \:\  \        \:\/:/  /     \:\  \                     
    \::/  /       \:\__\          \:\__\    \:\__\      \:\__\        \::/  /       \:\__\                    
     \/__/         \/__/           \/__/     \/__/       \/__/         \/__/         \/__/                    
      ___           ___                         ___                                     ___           ___     
     /\  \         /\  \         _____         /\  \                                   /\__\         /\__\    
    /::\  \       /::\  \       /::\  \       /::\  \         ___         ___         /:/  /        /:/ _/_   
   /:/\:\__\     /:/\:\  \     /:/\:\  \     /:/\:\  \       /\__\       /\__\       /:/  /        /:/ /\  \  
  /:/ /:/  /    /:/  \:\  \   /:/ /::\__\   /:/  \:\  \     /:/  /      /:/__/      /:/  /  ___   /:/ /::\  \ 
 /:/_/:/__/___ /:/__/ \:\__\ /:/_/:/\:|__| /:/__/ \:\__\   /:/__/      /::\  \     /:/__/  /\__\ /:/_/:/\:\__\ 
 \:\/:::::/  / \:\  \ /:/  / \:\/:/ /:/  / \:\  \ /:/  /  /::\  \      \/\:\  \__  \:\  \ /:/  / \:\/:/ /:/  /
  \::/~~/~~~~   \:\  /:/  /   \::/_/:/  /   \:\  /:/  /  /:/\:\  \      ~~\:\/\__\  \:\  /:/  /   \::/ /:/  / 
   \:\~~\        \:\/:/  /     \:\/:/  /     \:\/:/  /   \/__\:\  \        \::/  /   \:\/:/  /     \/_/:/  /  
    \:\__\        \::/  /       \::/  /       \::/  /         \:\__\       /:/  /     \::/  /        /:/  /   
     \/__/         \/__/         \/__/         \/__/           \/__/       \/__/       \/__/         \/__/    
'''

import wpilib
from math import fabs
import logging

class Gemini(wpilib.TimedRobot):

    def __init__(self):
        """ Initialization of internal class variables and software-bases only """

        super().__init__()

        # global button status list construction
        self.buttonToggleStatus = [False, False, False, False, False, False, False]

        from networktables import NetworkTables

        # connection for logging & Smart Dashboard
        self.sd = NetworkTables.getTable('SmartDashboard')
        NetworkTables.initialize(server='10.55.49.2')
        self.sd.putString("  ", "Connection")

    def robotInit(self):
        ''' Initialization of robot systems. '''

        logging.info('-( ͡° ͜ʖ ͡°)╯╲_-^o=o\ \"Don\'t mind me, just walking the robot.\"')

        from wpilib.drive import DifferentialDrive
        from ctre import WPI_TalonSRX, WPI_VictorSPX

        # drive train motors
        self.frontRightMotor = WPI_TalonSRX(4)
        self.rearRightMotor = WPI_TalonSRX(3)
        self.frontLeftMotor = WPI_TalonSRX(1)
        self.rearLeftMotor = WPI_TalonSRX(2)

        # lift encoder construction
        self.liftEncoder = wpilib.Encoder(8, 9)

        # liftArm encoder construction
        self.liftArmEncoder = wpilib.Encoder(5, 6)

        # drive train motor groups assignment
        self.left = wpilib.SpeedControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.SpeedControllerGroup(self.frontRightMotor, self.rearRightMotor)

        # drive train drive group assignment
        self.drive = DifferentialDrive(self.left, self.right)
        self.drive.setExpiration(0.1)

        # lift motor system initialization
        self.liftOne = WPI_VictorSPX(1)
        self.liftTwo = WPI_VictorSPX(2)
        self.lift = wpilib.SpeedControllerGroup(self.liftOne, self.liftTwo)

        # lift arm motor system initialization
        self.liftArmOne = WPI_VictorSPX(3)
        self.liftArmTwo = WPI_VictorSPX(4)
        self.liftArm = wpilib.SpeedControllerGroup(self.liftArmOne, self.liftArmTwo)

        # cargo intake motor initialization
        self.cargo = WPI_VictorSPX(5)

        # game and joystick controller construction
        # joystick - 0, 1 | controller - 2
        self.leftStick = wpilib.Joystick(0)
        self.rightStick = wpilib.Joystick(1)
        self.xbox = wpilib.Joystick(2)
        self.buttonBox = wpilib.Joystick(3)

        # pneumatic and compressor system initialization
        self.Compressor = wpilib.Compressor(0)
        self.Compressor.setClosedLoopControl(True)
        self.enable = self.Compressor.getPressureSwitchValue()
        self.DoubleSolenoidOne = wpilib.DoubleSolenoid(0, 1)    # gear shifting
        self.DoubleSolenoidTwo = wpilib.DoubleSolenoid(2, 3)    # hatch panel claw
        self.DoubleSolenoidThree = wpilib.DoubleSolenoid(4, 5)  # hatch panel ejection
        self.Compressor.start()

        # Smart Dashboard and NetworkTables initialization and construction
        self.PDP = wpilib.PowerDistributionPanel()
        self.roboController = wpilib.RobotController()
        self.DS = wpilib.DriverStation.getInstance()

        # proximity detection sensors
        self.Hall = wpilib.DigitalInput(7)
        self.ultrasonic = wpilib.AnalogInput(2)
        self.cargoUltrasonic = wpilib.AnalogInput(3)

        # timer construction
        self.timer = wpilib.Timer()

        # initialization of the HTTP camera
        wpilib.CameraServer.launch('vision.py:main')
        self.sd.putString("", "Top Camera")
        self.sd.putString(" ", "Bottom Camera")

        from sensors import REV_Color_Sensor_V2

        # Initialization and configuration of I2C interface with color sensor.
        self.colorSensor = REV_Color_Sensor_V2(wpilib.I2C.Port.kOnboard)

    def autonomousInit(self):
        ''' Executed each time the robot enters autonomous. '''

        # pre-auto timer configuration
        self.timer.reset()
        self.timer.start()

        # drive train encoder reset
        self.frontRightMotor.setQuadraturePosition(0, 0)
        self.frontLeftMotor.setQuadraturePosition(0, 0)

        self.liftEncoder.reset()

    def autonomousPeriodic(self):
        '''' Called periodically during autonomous. '''

        if self.DS.getGameSpecificMessage():
            # Test Methods
            if self.DS.getGameSpecificMessage() is 'encoder_test':

                # Drives robot set encoder distance away
                self.rightPos = fabs(self.frontRightMotor.getQuadraturePosition())
                self.leftPos = fabs(self.frontLeftMotor.getQuadraturePosition())
                self.distIn = (((self.leftPos + self.rightPos) / 2) / 4096) * 18.84955
                if 0 <= self.distIn <= 72:
                    self.drive.tankDrive(0.5, 0.5)
                else:
                    self.drive.tankDrive(0, 0)

            if self.DS.getGameSpecificMessage() is 'diagnostics':

                # Smart Dashboard Tests
                self.sd.putNumber("Temperature: ", self.PDP.getTemperature())
                self.sd.putNumber("Battery Voltage: ", self.roboController.getBatteryVoltage())
                self.sd.putBoolean(" Browned Out?", self.roboController.isBrownedOut)

                # Smart Dashboard diagnostics
                self.sd.putNumber("Right Encoder Speed: ", abs(self.frontRightMotor.getQuadratureVelocity()))
                self.sd.putNumber("Left Encoder Speed: ", abs(self.frontLeftMotor.getQuadratureVelocity()))
                self.sd.putNumber("Lift Encoder: ", self.liftEncoder.getDistance())

            if self.DS.getGameSpecificMessage() is 'pressure':
                self.Compressor.start()
            elif self.Compressor.enabled() is True:
                self.Compressor.stop()

        if not self.DS.getGameSpecificMessage() is 'pressure' and not self.DS.getGameSpecificMessage() is 'encoder_test':
            # begin normal periodic

            # get all required data once per frame
            # toggle button management per frame
            if self.buttonBox.getRawButtonPressed(1):
                self.buttonToggleStatus = [not self.buttonToggleStatus[0], False, False, False, False, False, False]
            elif self.buttonBox.getRawButtonPressed(2):
                self.buttonToggleStatus = [False, not self.buttonToggleStatus[1], False, False, False, False, False]
            elif self.buttonBox.getRawButtonPressed(3):
                self.buttonToggleStatus = [False, False, not self.buttonToggleStatus[2], False, False, False, False]
            elif self.buttonBox.getRawButtonPressed(4):
                self.buttonToggleStatus = [False, False, False, not self.buttonToggleStatus[3], False, False, False]
            elif self.buttonBox.getRawButtonPressed(5):
                self.buttonToggleStatus = [False, False, False, False, not self.buttonToggleStatus[4], False, False]
            elif self.buttonBox.getRawButtonPressed(6):
                self.buttonToggleStatus = [False, False, False, False, False, not self.buttonToggleStatus[5], False]
            elif self.buttonBox.getRawButtonPressed(7):
                self.buttonToggleStatus = [False, False, False, False, False, False, not self.buttonToggleStatus[6]]

            liftTicks = self.liftEncoder.get()
            hallState = self.Hall.get()
            compressorState = self.Compressor.enabled()
            solenoidStateOne = self.DoubleSolenoidOne.get()
            solenoidStateTwo = self.DoubleSolenoidTwo.get()
            solenoidStateThree = self.DoubleSolenoidThree.get()

            # robot ultrasonic
            self.ultraValue = self.ultrasonic.getVoltage()

            # cargo ultrasonic
            self.cargoUltraValue = self.cargoUltrasonic.getVoltage()

            # xbox value states
            xboxButtonStates = [self.xbox.getRawButton(1), self.xbox.getRawButton(2), self.xbox.getRawButton(3), self.xbox.getRawButton(4), self.xbox.getRawButton(5), self.xbox.getRawButton(6), self.xbox.getRawButton(7), self.xbox.getRawButton(8), self.xbox.getRawButton(9), self.xbox.getRawButton(10)]
            xboxAxisStates = [self.xbox.getRawAxis(1), self.xbox.getRawAxis(2), self.xbox.getRawAxis(3), self.xbox.getRawAxis(4), self.xbox.getRawAxis(5)]

            # joystick value states
            rJoystickButtonStates = [self.rightStick.getRawButton(1)]
            rJoystickAxisStates = [self.rightStick.getRawAxis(1), self.rightStick.getRawAxis(2), self.rightStick.getRawAxis(3)]
            lJoystickButtonStates = [self.leftStick.getRawButton(1)]
            lJoystickAxisStates = [self.leftStick.getRawAxis(1), self.leftStick.getRawAxis(2), self.leftStick.getRawAxis(3)]

            # define lift stages
            def cargo_one():
                if liftTicks <= 133:  # Cargo 1
                    self.lift.set(0.5)
                elif liftTicks > 133:
                    self.lift.set(0.05)

            def cargo_two():
                if liftTicks <= 270:   # Cargo 2
                    self.lift.set(0.5)
                elif liftTicks > 270:
                    self.lift.set(0.05)

            def cargo_three():
                if liftTicks <= 415:   # Cargo 3
                    self.lift.set(0.5)
                elif liftTicks > 415:
                    self.lift.set(0.05)

            def hatch_one():
                if liftTicks <= 96:    # Hatch 1
                    self.lift.set(0.5)
                elif liftTicks > 96:
                    self.lift.set(0.05)

            def hatch_two():
                if liftTicks <= 237:   # Hatch 2
                    self.lift.set(0.5)
                elif liftTicks > 237:
                    self.lift.set(0.05)

            def hatch_three():
                if liftTicks <= 378:   # Hatch 3
                    self.lift.set(0.5)
                elif liftTicks > 378:
                    self.lift.set(0.05)

            def lift_encoder_reset():
                self.lift.set(0.01)
                if hallState is True:
                    self.liftEncoder.reset()

            if self.buttonToggleStatus[0] is True:
                cargo_three()
            elif self.buttonToggleStatus[1] is True:
                hatch_three()
            elif self.buttonToggleStatus[2] is True:
                cargo_two()
            elif self.buttonToggleStatus[3] is True:
                hatch_two()
            elif self.buttonToggleStatus[4] is True:
                cargo_one()
            elif self.buttonToggleStatus[5] is True:
                hatch_one()
            elif self.buttonToggleStatus[6] is True:
                lift_encoder_reset()

            # compressor state
            self.sd.putString("Compressor Status: ", "Enabled" if compressorState is True else "Disabled")

            # gear state
            self.sd.putString("Gear Shift: ", "High Speed" if solenoidStateOne is 1 else "Low Speed")

            # ejector state
            self.sd.putString("Ejector Pins: ", "Ejected" if solenoidStateThree is 2 else "Retracted")

            # claw state
            self.sd.putString("Claw: ", "Open" if solenoidStateTwo is 2 else "Closed")

            # hatch station range state
            self.sd.putString("PLAYER STATION RANGE: ", "YES!!!!" if 0.142 <= self.ultraValue <= 0.146 else "NO!")
            # self.sd.putNumber("Ultrasonic Voltage: ", self.ultraValue)

            # hatch spaceship range
            self.sd.putString("HATCH RANGE: ", "HATCH IN RANGE" if 0.7 <= self.cargoUltraValue <= 1.56 else "NOT IN RANGE")

            # compressor
            if xboxButtonStates[8]:
                self.Compressor.stop()
            elif xboxButtonStates[9]:
                self.Compressor.start()
            elif rJoystickButtonStates[0]:  # shift right
                self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)
            elif lJoystickButtonStates[0]:  # shift left
                self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)
            elif xboxButtonStates[2]:  # open claw
                self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)
            elif xboxButtonStates[1]:  # close claw
                self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)
            elif xboxButtonStates[3]:  # eject
                self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)
            elif xboxButtonStates[0]:  # retract
                self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)

            # lift control
            if True in self.buttonToggleStatus is False:
                if xboxButtonStates[4]:  # hold
                    self.lift.set(0.05)
                elif xboxAxisStates[2] > 0.1:  # up
                    self.lift.set(xboxAxisStates[2] / 1.5)
                elif xboxAxisStates[1] > 0.1:  # down
                    self.lift.set(-xboxAxisStates[1] * 0.25)
                else:
                    self.lift.set(0)

            # four-bar control
            if xboxButtonStates[5]:
               self.liftArm.set(0.05)
            elif not xboxButtonStates[5]:
                self.liftArm.set(-xboxAxisStates[0] / 4.0)
            else:
                self.liftArm.set(0)

            # cargo intake control
            if xboxButtonStates[6]:
                self.cargo.set(0.12)
            elif xboxAxisStates[4]:  # take in
                self.cargo.set(xboxAxisStates[4] *0.75)

            # controller mapping for tank steering
            rightAxis = rJoystickAxisStates[0]
            leftAxis = lJoystickAxisStates[0]

            # drives drive system using tank steering
            if solenoidStateOne is 1:  # if on high gear
                divisor = 1.2  # then 90% of high speed
            elif solenoidStateOne is 2:  # if on low gear
                divisor = 1.2  # then normal slow speed
            else:
                divisor = 1.0

            leftSign = leftAxis / fabs(leftAxis) if leftAxis != 0 else 0
            rightSign = rightAxis / fabs(rightAxis) if rightAxis != 0 else 0

            self.drive.tankDrive(-leftSign * (1 / divisor) * (leftAxis ** 2), -rightSign * (1 / divisor) * (rightAxis ** 2))

    def teleopInit(self):
        ''' Executed at the start of teleop mode. '''

        self.drive.setSafetyEnabled(True)

        # drive train encoder reset
        self.frontRightMotor.setQuadraturePosition(0, 0)
        self.frontLeftMotor.setQuadraturePosition(0, 0)

        # lift encoder rest
        self.liftEncoder.reset()

        # compressor
        self.Compressor.start()

    def teleopPeriodic(self):
        ''' Periodically executes methods during the teleop mode. '''

        # begin normal periodic

        # get all required data once per frame
        # toggle button management per frame
        if self.buttonBox.getRawButtonPressed(1):
            self.buttonToggleStatus = [not self.buttonToggleStatus[0], False, False, False, False, False, False]
        elif self.buttonBox.getRawButtonPressed(2):
            self.buttonToggleStatus = [False, not self.buttonToggleStatus[1], False, False, False, False, False]
        elif self.buttonBox.getRawButtonPressed(3):
            self.buttonToggleStatus = [False, False, not self.buttonToggleStatus[2], False, False, False, False]
        elif self.buttonBox.getRawButtonPressed(4):
            self.buttonToggleStatus = [False, False, False, not self.buttonToggleStatus[3], False, False, False]
        elif self.buttonBox.getRawButtonPressed(5):
            self.buttonToggleStatus = [False, False, False, False, not self.buttonToggleStatus[4], False, False]
        elif self.buttonBox.getRawButtonPressed(6):
            self.buttonToggleStatus = [False, False, False, False, False, not self.buttonToggleStatus[5], False]
        elif self.buttonBox.getRawButtonPressed(7):
            self.buttonToggleStatus = [False, False, False, False, False, False, not self.buttonToggleStatus[6]]

        liftTicks = self.liftEncoder.get()
        hallState = self.Hall.get()
        compressorState = self.Compressor.enabled()
        solenoidStateOne = self.DoubleSolenoidOne.get()
        solenoidStateTwo = self.DoubleSolenoidTwo.get()
        solenoidStateThree = self.DoubleSolenoidThree.get()

        # robot ultrasonic
        self.ultraValue = self.ultrasonic.getVoltage()

        # cargo ultrasonic
        self.cargoUltraValue = self.cargoUltrasonic.getVoltage()

        # xbox value states
        xboxButtonStates = [self.xbox.getRawButton(1), self.xbox.getRawButton(2), self.xbox.getRawButton(3), self.xbox.getRawButton(4), self.xbox.getRawButton(5), self.xbox.getRawButton(6), self.xbox.getRawButton(7), self.xbox.getRawButton(8), self.xbox.getRawButton(9), self.xbox.getRawButton(10)]
        xboxAxisStates = [self.xbox.getRawAxis(1), self.xbox.getRawAxis(2), self.xbox.getRawAxis(3), self.xbox.getRawAxis(4), self.xbox.getRawAxis(5)]

        # joystick value states
        rJoystickButtonStates = [self.rightStick.getRawButton(1)]
        rJoystickAxisStates = [self.rightStick.getRawAxis(1), self.rightStick.getRawAxis(2), self.rightStick.getRawAxis(3)]
        lJoystickButtonStates = [self.leftStick.getRawButton(1)]
        lJoystickAxisStates = [self.leftStick.getRawAxis(1), self.leftStick.getRawAxis(2), self.leftStick.getRawAxis(3)]

        # define lift stages
        def cargo_one():
            if liftTicks <= 133:  # Cargo 1
                self.lift.set(0.5)
            elif liftTicks > 133:
                self.lift.set(0.05)

        def cargo_two():
            if liftTicks <= 270:  # Cargo 2
                self.lift.set(0.5)
            elif liftTicks > 270:
                self.lift.set(0.05)

        def cargo_three():
            if liftTicks <= 415:  # Cargo 3
                self.lift.set(0.5)
            elif liftTicks > 415:
                self.lift.set(0.05)

        def hatch_one():
            if liftTicks <= 96:  # Hatch 1
                self.lift.set(0.5)
            elif liftTicks > 96:
                self.lift.set(0.05)

        def hatch_two():
            if liftTicks <= 237:  # Hatch 2
                self.lift.set(0.5)
            elif liftTicks > 237:
                self.lift.set(0.05)

        def hatch_three():
            if liftTicks <= 378:  # Hatch 3
                self.lift.set(0.5)
            elif liftTicks > 378:
                self.lift.set(0.05)

        def lift_encoder_reset():
            self.lift.set(0.01)
            if hallState is True:
                self.liftEncoder.reset()

        if self.buttonToggleStatus[0] is True:
            cargo_three()
        elif self.buttonToggleStatus[1] is True:
            hatch_three()
        elif self.buttonToggleStatus[2] is True:
            cargo_two()
        elif self.buttonToggleStatus[3] is True:
            hatch_two()
        elif self.buttonToggleStatus[4] is True:
            cargo_one()
        elif self.buttonToggleStatus[5] is True:
            hatch_one()
        elif self.buttonToggleStatus[6] is True:
            lift_encoder_reset()

        # compressor state
        self.sd.putString("Compressor Status: ", "Enabled" if compressorState is True else "Disabled")

        # gear state
        self.sd.putString("Gear Shift: ", "High Speed" if solenoidStateOne is 1 else "Low Speed")

        # ejector state
        self.sd.putString("Ejector Pins: ", "Ejected" if solenoidStateThree is 2 else "Retracted")

        # claw state
        self.sd.putString("Claw: ", "Open" if solenoidStateTwo is 2 else "Closed")

        # hatch station range state
        self.sd.putString("PLAYER STATION RANGE: ", "YES!!!!" if 0.142 <= self.ultraValue <= 0.146 else "NO!")
        # self.sd.putNumber("Ultrasonic Voltage: ", self.ultraValue)

        # hatch spaceship range
        self.sd.putString("HATCH RANGE: ", "HATCH IN RANGE" if 0.7 <= self.cargoUltraValue <= 1.56 else "NOT IN RANGE")

        # compressor
        if xboxButtonStates[8]:
            self.Compressor.stop()
        elif xboxButtonStates[9]:
            self.Compressor.start()
        elif rJoystickButtonStates[0]:  # shift right
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kForward)
        elif lJoystickButtonStates[0]:  # shift left
            self.DoubleSolenoidOne.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif xboxButtonStates[2]:  # open claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kForward)
        elif xboxButtonStates[1]:  # close claw
            self.DoubleSolenoidTwo.set(wpilib.DoubleSolenoid.Value.kReverse)
        elif xboxButtonStates[3]:  # eject
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kForward)
        elif xboxButtonStates[0]:  # retract
            self.DoubleSolenoidThree.set(wpilib.DoubleSolenoid.Value.kReverse)

        # lift control
        if True in self.buttonToggleStatus is False:
            if xboxButtonStates[4]:  # hold
                self.lift.set(0.05)
            elif xboxAxisStates[2] > 0.1:  # up
                self.lift.set(xboxAxisStates[2] / 1.5)
            elif xboxAxisStates[1] > 0.1:  # down
                self.lift.set(-xboxAxisStates[1] * 0.25)
            else:
                self.lift.set(0)

        # four-bar control
        if xboxButtonStates[5]:
            self.liftArm.set(0.05)
        elif not xboxButtonStates[5]:
            self.liftArm.set(-xboxAxisStates[0] / 4.0)
        else:
            self.liftArm.set(0)

        # cargo intake control
        if xboxButtonStates[6]:
            self.cargo.set(0.12)
        elif xboxAxisStates[4]:  # take in
            self.cargo.set(xboxAxisStates[4] * 0.75)

        # controller mapping for tank steering
        rightAxis = rJoystickAxisStates[0]
        leftAxis = lJoystickAxisStates[0]

        # drives drive system using tank steering
        if solenoidStateOne is 1:  # if on high gear
            divisor = 1.2  # then 90% of high speed
        elif solenoidStateOne is 2:  # if on low gear
            divisor = 1.2  # then normal slow speed
        else:
            divisor = 1.0

        leftSign = leftAxis / fabs(leftAxis) if leftAxis != 0 else 0
        rightSign = rightAxis / fabs(rightAxis) if rightAxis != 0 else 0

        self.drive.tankDrive(-leftSign * (1 / divisor) * (leftAxis ** 2), -rightSign * (1 / divisor) * (rightAxis ** 2))


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    wpilib.run(Gemini)
