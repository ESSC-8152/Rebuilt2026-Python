from wpilib import XboxController, Compressor, PneumaticsModuleType
from wpimath import applyDeadband

from constants import OIConstants, DriveConstants
from subsystems.conduireSubsystem import conduireSubsystem
from commands2 import RunCommand, button, Command


class RobotContainer:
    def __init__(self):
        # initialisation des subsystems
        self.m_conduire = conduireSubsystem()

        # Initialisation des manettes
        self.m_mannetteDriver = XboxController(OIConstants.kPortManetteConducteur)
        self.m_mannetteCopilote = XboxController(OIConstants.kPortManetteCopilote)

        # Setup de la commande par défaut pour la conduite
        self.m_conduire.setDefaultCommand(
            RunCommand(
               lambda: self.m_conduire.conduire(
                   applyDeadband(self.m_mannetteDriver.getLeftY(), OIConstants.kConduireDeadband),
                   applyDeadband(self.m_mannetteDriver.getLeftX(), OIConstants.kConduireDeadband),
                   applyDeadband(self.m_mannetteDriver.getRightX(), OIConstants.kConduireDeadband),
                    True,
                    True
            ),self.m_conduire
        ))

        # Configuration du compresseur
        self.m_compresseur = Compressor(PneumaticsModuleType.CTREPCM)
        self.m_compresseur.enableDigital()

        # Configuration des boutons
        self.configurerboutons()

    def configurerboutons(self):
        # Bouton R1 pour mettre le robot en formation X (break)
        button.JoystickButton(self.m_mannetteDriver, XboxController.Button.kRightBumper).whileTrue(
            RunCommand(
                lambda: self.m_conduire.setFormationArret(),
                self.m_conduire
            )
        )