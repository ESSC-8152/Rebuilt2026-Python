# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import math
from rev import SparkMaxConfig
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import inchesToMeters

class NeoMotorConstants:
    kFreeSpeedRpm = 5676

class DriveConstants:
    kMultiplicateurVitesse = 0.6

    # Paramêtre de driving - c'est pas vraiment la vitesse max du robot juste max allouée
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = 2 * math.pi  # radians par seconde

    kDirectionSlewRate = 1.2  # radians par seconde
    kMagnitudeSlewRate = 1.8  # pourcentage par seconde (1 = 100%)
    kRotationalSlewRate = 2.0  # pourcentage par seconde (1 = 100%)

    # Configuration du châssis
    kTrackWidth = inchesToMeters(26.5)  # Distance entre les centres des roues droite et gauche sur le robot (mètres)
    kWheelBase = inchesToMeters(26.5)  # Distance entre les roues avant et arrière sur le robot (mètres)

    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kWheelBase / 2, kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    )

    # Angular offsets of the modules relative to the chassis in radians
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = math.pi
    kBackRightChassisAngularOffset = math.pi / 2

    # SPARK MAX CAN IDs
    kFrontLeftDrivingCanId = 7
    kRearLeftDrivingCanId = 5
    kFrontRightDrivingCanId = 1
    kRearRightDrivingCanId = 3

    kFrontLeftTurningCanId = 8
    kRearLeftTurningCanId = 6
    kFrontRightTurningCanId = 2
    kRearRightTurningCanId = 4

    kGyroReversed = True


class ModuleConstants:
    # Les swerves peuvent être configurés avec trois engrenages pignon: 12T, 13T, ou 14T.
    # Ça change la vitesse de conduite du module (un pignon avec plus de dents va faire un
    # robot qui conduit plus rapidement).
    kDrivingMotorPinionTeeth = 13

    # Inverser l'encodeur de rotation, vu que le shaft tourne dans la direction opposée du
    # moteur de direction dans un Swerve.
    kTurningEncoderInverted = True

    # Calculs nécessaires pour les facteurs de conversion du moteur de conduite et l'avance
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = inchesToMeters(3)  # 3 pouces de diamètre de roue
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 dents sur l'engrenage conique de la roue, 22 dents sur l'engrenage droit de première étape, 15 dents sur le pignon conique (google translate qui carry)
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction  # mêtres
    kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) / 60.0  # mètres par seconde

    kTurningEncoderPositionFactor = (2 * math.pi)  # radians
    kTurningEncoderVelocityFactor = (2 * math.pi) / 60.0  # radians par seconde
    kTurningEncoderPositionPIDMinInput = 0  # radians
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radians

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = SparkMaxConfig.IdleMode.kBrake  # peut être "brake" ou "coast"
    kTurningMotorIdleMode = SparkMaxConfig.IdleMode.kBrake  # peut être "brake" ou "coast"

    kDrivingMotorCurrentLimit = 50  # ampères
    kTurningMotorCurrentLimit = 20  # ampères
    
class OIConstants:
    # À vérifier avec les valeurs du Driver Station
    kPortManetteConducteur = 0
    kPortManetteCopilote = 1
    kConduireDeadband = 0.05

class AutoConstants:
    kMaxVitesseMetresParSecondes = 3
    kMaxAccelerationMetresParSecondesCarre = 3
    kMaxVitesseAngulaireRadiansParSeconde = math.pi
    kMaxVitesseAngulaireRadiansParSecondeCarre = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    # Contraintes pour le contrôleur d'angle profilé en mouvement du robot
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxVitesseAngulaireRadiansParSeconde, kMaxVitesseAngulaireRadiansParSecondeCarre
    )


