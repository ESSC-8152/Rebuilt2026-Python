from rev import SparkMax
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from Configs import MAXSwerveModule as SparkMaxConfig

class MAXSwerveModule:
    def __init__(self, drivingCanId, turningCanId, chassisAngularOffset):
        # Initialisation des composants du swerve
        self.m_drivingSpark = SparkMax(drivingCanId, SparkMax.MotorType.kBrushless)
        self.m_turningSpark = SparkMax(turningCanId, SparkMax.MotorType.kBrushless)
        
        self.m_drivingEncoder = self.m_drivingSpark.getEncoder()
        self.m_turningEncoder = self.m_turningSpark.getAbsoluteEncoder()
        
        self.m_drivingClosedLoopController = self.m_drivingSpark.getClosedLoopController()
        self.m_turningClosedLoopController = self.m_turningSpark.getClosedLoopController()
        
        self.m_drivingSpark.configure(SparkMaxConfig.drivingConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)
        self.m_turningSpark.configure(SparkMaxConfig.turningConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)
        
        self.m_chassisAngularOffset = chassisAngularOffset
        # Etat désiré initial (immobile, angle actuel corrigé par l'offset)
        self.m_desiredState = SwerveModuleState(0.0, Rotation2d(self.m_turningEncoder.getPosition() - self.m_chassisAngularOffset))
        self.m_drivingEncoder.setPosition(0.0)
        
    def getState(self):
        # Retourne l'état actuel du swerve
        return SwerveModuleState(self.m_drivingEncoder.getVelocity(), Rotation2d(self.m_turningEncoder.getPosition() - self.m_chassisAngularOffset))
    
    def getPosition(self):
        # Retourne la position actuelle du swerve
        return SwerveModulePosition(self.m_drivingEncoder.getPosition(), Rotation2d(self.m_turningEncoder.getPosition() - self.m_chassisAngularOffset))
    
    def setDesiredState(self, desiredState: SwerveModuleState):
        """Applique l'état désiré au module swerve.
        Sécurise contre les états None et optimise la rotation pour minimiser le mouvement.
        """
        if desiredState is None:
            return

        # Appliquer l'offset châssis à l'angle demandé
        correctedDesiredState = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.m_chassisAngularOffset)
        )

        # Angle actuel du module (après offset déjà pris en compte dans getPosition/getState)
        currentAngle = Rotation2d(self.m_turningEncoder.getPosition())
        optimizedState = SwerveModuleState.optimize(correctedDesiredState, currentAngle)

        # Certaines implémentations peuvent retourner None: fallback
        if optimizedState is None:
            optimizedState = correctedDesiredState

        # Commander les PID internes
        self.m_drivingClosedLoopController.setReference(optimizedState.speed, SparkMax.ControlType.kVelocity)
        self.m_turningClosedLoopController.setReference(optimizedState.angle.radians(), SparkMax.ControlType.kPosition)

        self.m_desiredState = optimizedState
        
    def resetEncoders(self):
        # Réinitialise les encodeurs du swerve
        self.m_drivingEncoder.setPosition(0.0)