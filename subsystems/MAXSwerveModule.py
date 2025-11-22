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
        self.m_desiredState = Rotation2d(self.m_turningEncoder.getPosition())
        self.m_drivingEncoder.setPosition(0.0)
        
    def getState(self):
        # Retourne l'état actuel du swerve
        return SwerveModuleState(self.m_drivingEncoder.getVelocity(), Rotation2d(self.m_turningEncoder.getPosition() - self.m_chassisAngularOffset))
    
    def getPosition(self):
        # Retourne la position actuelle du swerve
        return SwerveModulePosition(self.m_drivingEncoder.getPosition(), Rotation2d(self.m_turningEncoder.getPosition() - self.m_chassisAngularOffset))
    
    def setDesiredState(self, desiredState):
        
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(self.m_chassisAngularOffset)
        
        # Optimize retourne un nouveau state optimisé
        optimizedState = SwerveModuleState.optimize(correctedDesiredState, Rotation2d(self.m_turningEncoder.getPosition()))
        
        self.m_drivingClosedLoopController.setReference(optimizedState.speed, SparkMax.ControlType.kVelocity)
        self.m_turningClosedLoopController.setReference(optimizedState.angle.radians(), SparkMax.ControlType.kPosition)
        
        self.m_desiredState = desiredState
        
    def resetEncoders(self):
        # Réinitialise les encodeurs du swerve
        self.m_drivingEncoder.setPosition(0.0)