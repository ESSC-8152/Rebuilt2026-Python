import MAXSwerveModule

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import SwerveDriveKinematics
from wpimath.geometry import Rotation2d, Pose2d
from wpilib import Field2d, SmartDashboard
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
        
    def getAngle(self):
        # Retourne l'angle actuel du robot à partir du gyroscope
        return self.navx.getAngle()
    
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
         
        