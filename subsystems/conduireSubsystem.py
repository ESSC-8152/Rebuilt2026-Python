from subsystems import MAXSwerveModule

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
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

        self.multiplicateurVitesse = DriveConstants.kMultiplicateurVitesse
        
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

        self.m_kinematics = DriveConstants.kDriveKinematics

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
        
    def modifierEtatSwerves(self, etatsFinaux):
        # Définit les états finaux désirés pour chaque module swerve
        self.avantGauche.setDesiredState(etatsFinaux[0])
        self.avantDroit.setDesiredState(etatsFinaux[1])
        self.arriereGauche.setDesiredState(etatsFinaux[2])
        self.arriereDroit.setDesiredState(etatsFinaux[3])
        
    def getEtatsSwerves(self):
        # Retourne les états actuels de chaque module swerve
        return (
            self.avantGauche.getState(),
            self.avantDroit.getState(),
            self.arriereGauche.getState(),
            self.arriereDroit.getState()
        )
    
    def conduire(self, vitesseX, vitesseY, rot, fieldRelative, squared):
        deadband = 0.05
        # appliquer deadband
        vitesseX = applyDeadband(vitesseX, deadband)
        vitesseY = applyDeadband(vitesseY, deadband)
        rot = applyDeadband(rot, deadband)
        
        # appliquer squared si jamais ce l'est
        if squared:
            vitesseX *= abs(vitesseX)
            vitesseY *= abs(vitesseY)
            rot *= abs(rot)

        vitesseX *= DriveConstants.kMaxSpeedMetersPerSecond
        vitesseY *= DriveConstants.kMaxSpeedMetersPerSecond
        rot *= DriveConstants.kMaxAngularSpeed

        # Application du multiplicateur de vitesse
        vitesseX *= self.multiplicateurVitesse
        vitesseY *= self.multiplicateurVitesse
        rot *= self.multiplicateurVitesse

        if fieldRelative:
            vitesses = ChassisSpeeds.fromFieldRelativeSpeeds(
                vitesseX,
                vitesseY,
                rot,
                self.navx.getRotation2d()
            )
        else:
            vitesses = ChassisSpeeds(vitesseX, vitesseY, rot)
        
        module_states = self.m_kinematics.toSwerveModuleStates(vitesses)
        self.modifierEtatSwerves(module_states)

    def modifierMultiplicateurVitesse(self, multiplicateurVitesse):
        self.multiplicateurVitesse = multiplicateurVitesse

    def stop(self):
        # Arrête tous les modules swerve
        self.modifierEtatSwerves((
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0))
        ))
        
    def setFormationArret(self):
        # Configure les modules swerve en formation "X" pour stabiliser le robot
        self.avantGauche.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.avantDroit.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.arriereGauche.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.arriereDroit.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        
    # ------------------Pose estimator---------------------- #    
    def getPosition(self) -> Pose2d:
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
            self.avantGauche.getState(),
            self.avantDroit.getState(),
            self.arriereGauche.getState(),
            self.arriereDroit.getState()
        )
        
    def conduireChassis(self, chassisSpeeds: ChassisSpeeds):
        # Conduit le robot en utilisant les vitesses du châssis fournies par le PathPlanner
        targetSpeed = ChassisSpeeds.discretize(chassisSpeeds, 0.02)
        
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeed)
        self.modifierEtatSwerves(swerveModuleStates)
        
    def isRedAlliance(self) -> bool:
        # Détermine si le robot est dans l'alliance rouge
        ally = DriverStation.getAlliance()
        if ally == DriverStation.Alliance.kRed:
            return True
        return False