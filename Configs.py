import math

from rev import SparkMaxConfig, ClosedLoopConfig

from constants import ModuleConstants

class MAXSwerveModule:
    drivingConfig = SparkMaxConfig()
    turningConfig = SparkMaxConfig()
    
    drivingFactor = ModuleConstants.kWheelDiameterMeters * math.pi / ModuleConstants.kDrivingMotorReduction
    turningFactor = 2 * math.pi
    drivingVelocityFeedForward = 1 / (ModuleConstants.kDriveWheelFreeSpeedRps)
    
    drivingConfig.setIdleMode(ModuleConstants.kDrivingMotorIdleMode)
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
    
    drivingConfig.encoder.positionConversionFactor(drivingFactor)
    drivingConfig.encoder.velocityConversionFactor(drivingFactor / 60.0)
    
    drivingConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
    drivingConfig.closedLoop.pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
    drivingConfig.closedLoop.velocityFF(drivingVelocityFeedForward)
    drivingConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput)
    
    turningConfig.setIdleMode(ModuleConstants.kTurningMotorIdleMode)
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit)
    
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted)
    turningConfig.absoluteEncoder.positionConversionFactor(turningFactor)
    turningConfig.absoluteEncoder.velocityConversionFactor(turningFactor / 60.0)
    
    turningConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
    turningConfig.closedLoop.pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
    turningConfig.closedLoop.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
    turningConfig.closedLoop.positionWrappingEnabled(True)
    turningConfig.closedLoop.positionWrappingInputRange(0,turningFactor)