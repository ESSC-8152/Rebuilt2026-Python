from wpilib import XboxController, Compressor, PneumaticsModuleType
from wpimath import applyDeadband

from constants import OIConstants, DriveConstants
from subsystems.conduireSubsystem import conduireSubsystem
from commands2 import RunCommand, button, Command


class RobotContainer:
    def __init__(self):
        # init des subsystems
        self.m_conduire = conduireSubsystem()

        self.m_compressor = Compressor(PneumaticsModuleType.CTREPCM)

        self.m_mannetteDriver = XboxController(OIConstants.kDriverControllerPort)
        
        self.m_conduire.setDefaultCommand(
            RunCommand(
               lambda: self.m_conduire.conduire(
                    applyDeadband(self.m_mannetteDriver.getLeftY(), OIConstants.kDriveDeadband),
                    applyDeadband(self.m_mannetteDriver.getLeftX(), OIConstants.kDriveDeadband),
                    applyDeadband(self.m_mannetteDriver.getRightX(), OIConstants.kDriveDeadband),
                    True,
                    True
            ),self.m_conduire
        ))

        self.m_compressor.enableDigital()
        # Configure button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self):
        # Bouton R1 pour mettre le robot en formation X
        button.JoystickButton(self.m_mannetteDriver, XboxController.Button.kRightBumper).whileTrue(
            RunCommand(
                lambda: self.m_conduire.setXFormation(),
                self.m_conduire
            )
        )

        self.m_mannetteDriver.leftTrigger()