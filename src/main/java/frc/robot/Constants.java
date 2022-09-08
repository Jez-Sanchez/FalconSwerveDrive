// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants{

    // TRANSLATE ALL THE CONSTANTS FOR OUR SWERVE DRIVE CHASSIS
        public static final double kTrackWidth = Units.inchesToMeters(11.375);//Change the distance for this 
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(13.5625);//Change the distance for this
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),//Revise
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),//Revise
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),//Revise
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));//Revise

        public static final int kFrontLeftDriveMotorPort = 1;// Has been revised adjust the motor controllers at the lab
        public static final int kBackLeftDriveMotorPort = 5;// Has been revised adjust the motor controllers at the lab
        public static final int kFrontRightDriveMotorPort = 3;//Has been revised adjust the motor controllers at the lab
        public static final int kBackRightDriveMotorPort = 7;//Has been revised adjust the motor controllers at the lab

        public static final int kFrontLeftTurningMotorPort = 2;//Has been revised adjust the motor controllers at the lab
        public static final int kBackLeftTurningMotorPort = 6;//Has been revised adjust the motor controllers at the lab
        public static final int kFrontRightTurningMotorPort = 4;//Has been revised adjust the motor controllers at the lab
        public static final int kBackRightTurningMotorPort = 8;//Has been revised adjust the motor controllers at the lab

        public static final boolean kFrontLeftTurningEncoderReversed = false;//Revise when at the lab
        public static final boolean kBackLeftTurningEncoderReversed = false;//Revise when at the lab
        public static final boolean kFrontRightTurningEncoderReversed = false;//Revise when at the lab
        public static final boolean kBackRightTurningEncoderReversed = false;//Revise when at the lab

        public static final boolean kFrontLeftDriveEncoderReversed = false;//Revise when at the lab
        public static final boolean kBackLeftDriveEncoderReversed = false;//Revise when at the lab
        public static final boolean kFrontRightDriveEncoderReversed = false;//Revise when at the lab
        public static final boolean kBackRightDriveEncoderReversed = false;//Revise when at the lab

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;//Revise at the lab if needed
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;//Revise at the lab if needed
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;//Revise at the lab if needed
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;//Revise at the lab if needed

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;//Revise at the lab
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;//Revise at the lab
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;//Revise at the lab
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;//Revise ta the lab

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;//Revise at the lab
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;//Revise at the lab
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;//Revise at the lab
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;//Revise at the lab

        public static final double kPhysicalMaxSpeedMetersPerSecond = 8;//Revise
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;//Revise

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;//Revise
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //Revise
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 0.5;//Revise if needed check back at the video
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 0.5;//Revise if needed check back at the video
    }
    public static final class ModuleConstants{
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);//Has been revised double check with the 3D design or at the lab
        public static final double kDriveMotorGearRatio = 12 / 24;//Revise at the lab
        public static final double kTurningMotorGearRatio = 22 / 24;//Revise at the lab
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;//check to see if needs to be revised
    }
    public static final class OIConstants{
        public static final int kDriverControllerPort = 0;

        // public static final int kDriverYAxis = 1;
        // public static final int kDriverXAxis = 0;
        // public static final int kDriverRotAxis = 4;
        // public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;//Check to see if needs to be revised
    }
}
