import MAXSwerveModule

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import SwerveDriveKinematics, ChassisSpeeds
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import Field2d, SmartDashboard, DriverStation
from wpimath import applyDeadband
from navx import AHRS

from commands2 import Subsystem

from constants import DriveConstants

class conduireSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        
        # NavX
        self.navx = AHRS.create_spi()
        
        self.avantGauche = MAXSwerveModule.MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset)
        
        self.avantDroit = MAXSwerveModule.MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset)
        
        self.arriereGauche = MAXSwerveModule.MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset)
        
        self.arriereDroit = MAXSwerveModule.MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset)
        
        self.poseEstimateur = SwerveDrive4PoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(self.getAngle()),
            (
                self.avantGauche.getPosition(),
                self.avantDroit.getPosition(),
                self.arriereGauche.getPosition(),
                self.arriereDroit.getPosition()
            ),
            Pose2d()
        )
        
        self.field2d = Field2d()
        
        self.resetGyro()
        self.resetEncoders()
        self.resetOdometry()
    
    def periodic(self):
        # Met à jour l'estimateur de pose et le Field2d
        self.poseEstimateur.update(
            Rotation2d.fromDegrees(self.getAngle()),
            (
                self.avantGauche.getPosition(),
                self.avantDroit.getPosition(),
                self.arriereGauche.getPosition(),
                self.arriereDroit.getPosition()
            )
        )
        self.field2d.setRobotPose(self.poseEstimateur.getEstimatedPosition())
        
        SmartDashboard.putData("Terrain de jeu", self.field2d)
        
        SmartDashboard.putNumber("Angle du Gyro", self.getAngle())
        
    def setModulesStates(self, desiredStates):
        # Définit les états désirés pour chaque module swerve
        self.avantGauche.setDesiredState(desiredStates[0])
        self.avantDroit.setDesiredState(desiredStates[1])
        self.arriereGauche.setDesiredState(desiredStates[2])
        self.arriereDroit.setDesiredState(desiredStates[3])
        
    def getModulesStates(self):
        # Retourne les états actuels de chaque module swerve
        return (
            self.avantGauche.getState(),
            self.avantDroit.getState(),
            self.arriereGauche.getState(),
            self.arriereDroit.getState()
        )
    
    def conduire(self,xSpeed,ySpeed,rot,fieldRelative,squared):
        deadband = 0.05
        # appliquer deadband
        xSpeed = -applyDeadband(xSpeed, deadband)
        ySpeed = -applyDeadband(ySpeed, deadband)
        rot = -applyDeadband(rot, deadband)
        
        # appliquer squared si jamais ce l'est
        if (squared):
            xSpeed = xSpeed * abs(xSpeed)
            ySpeed = ySpeed * abs(ySpeed)
            rot = rot * abs(rot)
            
        xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond
        rotDelivered = rot * DriveConstants.kMaxAngularSpeedRadiansPerSecond
        
        invert = -1
        
        if fieldRelative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered * invert,
                ySpeedDelivered * invert,
                rotDelivered,
                self.getPose().getRotation()
            )
        else:
            speeds = ChassisSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered
            )
            
    def stop(self):
        # Arrête tous les modules swerve
        self.setModulesStates((
            MAXSwerveModule.SwerveModuleState(0, Rotation2d(0)),
            MAXSwerveModule.SwerveModuleState(0, Rotation2d(0)),
            MAXSwerveModule.SwerveModuleState(0, Rotation2d(0)),
            MAXSwerveModule.SwerveModuleState(0, Rotation2d(0))
        ))
        
    def setXFormation(self):
        # Configure les modules swerve en formation "X" pour stabiliser le robot
        self.avantGauche.setDesiredState(MAXSwerveModule.SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.avantDroit.setDesiredState(MAXSwerveModule.SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.arriereGauche.setDesiredState(MAXSwerveModule.SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.arriereDroit.setDesiredState(MAXSwerveModule.SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        
    # ------------------Pose estimator---------------------- #    
    def getPose(self) -> Pose2d:
        # Retourne la pose estimée actuelle du robot
        return self.poseEstimateur.getEstimatedPosition()
    
    def resetOdometry(self, pose=Pose2d()):
        # Réinitialise l'odométrie du robot à une pose spécifique
        self.resetEncoders()
        self.poseEstimateur.resetPosition(
            Rotation2d.fromDegrees(self.getAngle()),
            (
                self.avantGauche.getPosition(),
                self.avantDroit.getPosition(),
                self.arriereGauche.getPosition(),
                self.arriereDroit.getPosition()
            ),
            pose
        )
        
    # ------------------Encoders-------------------#
    def resetEncoders(self):
        # Réinitialise les encodeurs de tous les modules swerve
        self.avantGauche.resetEncoders()
        self.avantDroit.resetEncoders()
        self.arriereGauche.resetEncoders()
        self.arriereDroit.resetEncoders()
        
    # ------------------Gyro---------------------- #
    def getAngle(self) -> float:
        # Retourne l'angle actuel du robot à partir du gyroscope
        return self.navx.getAngle()
    
    def getRate(self) -> float:
        # Retourne le taux de rotation actuel du robot à partir du gyroscope
        return self.navx.getRate()
    
    def resetGyro(self):
        # Réinitialise le gyroscope à zéro
        self.navx.reset()
        
    # ------------------PathPlanner---------------------- #
    def getChassisSpeeds(self) -> ChassisSpeeds:
        # Retourne les vitesses du châssis pour le PathPlanner
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            self.avantDroit.getState(),
            self.avantGauche.getState(),
            self.arriereDroit.getState(),
            self.arriereGauche.getState()
        )
        
    def conduireChassis(self, chassisSpeeds: ChassisSpeeds):
        # Conduit le robot en utilisant les vitesses du châssis fournies par le PathPlanner
        targetSpeed = ChassisSpeeds.discretize(chassisSpeeds, 0.02)
        
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeed)
        self.setModulesStates(swerveModuleStates)
        
    def isRedAlliance(self) -> bool:
        # Détermine si le robot est dans l'alliance rouge
        ally = DriverStation.getAlliance()
        if ally == DriverStation.Alliance.kRed:
            return True
        return False