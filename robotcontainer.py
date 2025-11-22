from wpilib import XboxController
from constants import OIConstants

class RobotContainer:
    def __init__(self):
        # init des subsystems
        # self.driveSubsystem = DriveSubsystem()
        
        m_mannetteDriver = XboxController(OIConstants.kDriverControllerPort)
        
        # Configure button bindings
        self.configureButtonBindings()
        
    def configureButtonBindings(self):
        # Configuration des boutons pour les commandes
        pass