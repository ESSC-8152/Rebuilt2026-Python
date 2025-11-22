from wpilib import XboxController
from wpimath import applyDeadband
from constants import OIConstants
from subsystems.conduireSubsystem import conduireSubsystem
from commands2 import RunCommand, button

class RobotContainer:
    def __init__(self):
        # init des subsystems
        self.m_conduire = conduireSubsystem()
        
        self.m_mannetteDriver = XboxController(OIConstants.kDriverControllerPort)
        
        self.m_conduire.setDefaultCommand(
            RunCommand(
               lambda: self.m_conduire.conduire(
                    -applyDeadband(self.m_mannetteDriver.getLeftY(), OIConstants.kDriveDeadband)*0.1,
                    -applyDeadband(self.m_mannetteDriver.getLeftX(), OIConstants.kDriveDeadband)*0.1,
                    -applyDeadband(self.m_mannetteDriver.getRightX(), OIConstants.kDriveDeadband)*0.3,
                    True,
                    True
            ),self.m_conduire
        ))
        
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
        