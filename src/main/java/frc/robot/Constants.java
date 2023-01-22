// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public static final class chassisConstants{

        public static double deadband = 0.30;

        public static double kS = (0.667 / 12);//Test and revise if need
        public static double kV = (2.44 / 12);//Test and revise if need
        public static double kA = (0.27 / 12);//Test and revise if need

        public static double driveMotorGearRat = (6.54 / 1);
        public static double turnMotorGearRat = (11.8 / 1); 
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static double circumference = wheelDiameter * Math.PI;

        public static double maxSpeedMPS = 6.5;
        public static double maxTurnSpeed = 2.0;
        
        //Distance between the front and back wheels in inches
        public static final double wheelBase = (13.5625);
        //Distance between left and right wheels in inches
        public static final double trackWidth = (11.369);

        public static PIDController xController = new PIDController(0, 0, 0);
        public static PIDController yController = new PIDController(0, 0, 0);
        public static final TrapezoidProfile.Constraints thetaConstraints = new TrapezoidProfile.Constraints(Math.PI, Math.PI);
        public static ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0, thetaConstraints);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    }
    public static final class chassisSetUp{
        //Front Left Module
        public static final int fLeftDriveMotorPort = 1;
        public static final int fLeftTurnMotorPort = 2;
        public static final boolean isFrontLeftDriveMotorReverse = true;
        public static final boolean isFrontLeftTurnMotorReverse = true;
        public static final int fLeftAbsoluteEncoder = 0;
        public static final double frontLAngle = Units.radiansToDegrees(0.024654);
        public static final double frontLKP = 0.7;
        public static final double frontLKI = 0.0;
        public static final double frontLKD = 0.0;
        

        //Front Right Module
        public static final int fRightDriveMotorPort = 3;
        public static final int fRightTurnMotorPort = 4;
        public static final boolean isFrontRightDriveMotorReverse = true;
        public static final boolean isFrontRightTurnMotorReverse = true;
        public static final int fRightAbsoluteEncoder = 1;
        public static final double frontRAngle = Units.radiansToDegrees(0.172576);
        public static final double frontRKP = 0.6;
        public static final double frontRKI = 0.0;
        public static final double frontRKD = 0.0;

        //Back Right Module
        public static final int bRightDriveMotorPort = 5;
        public static final int bRightTurnMotorPort = 6;
        public static final boolean isBackRightDriveMotorReverse = true;
        public static final boolean isBackRightTurnMotorReverse = true;
        public static final int bRightAbsoluteEncoder = 2;
        public static final double backRAngle = Units.radiansToDegrees(0.221884);
        public static final double backRKP = 0.7;
        public static final double backRKI = 0.0;
        public static final double backRKD = 0.0;

        //Back Left Module
        public static final int bLeftDriveMotorPort = 7;
        public static final int bLeftTurnMotorPort = 8;
        public static final boolean isBackLeftDriveMotorReverse = true;
        public static final boolean isBackLeftTurnMotorReverse = true;
        public static final int bLeftAbsoluteEncoder = 3;
        public static final double backLAngle = Units.radiansToDegrees(0.073961);
        public static final double backLKP = 0.6;
        public static final double backLKI = 0.0;
        public static final double backLKD = 0.0;

        //Gyro
        public static final boolean invertedGyro = false;
    }
    public static final class VisionSetUp{
        public static final double cameraHeight_Meters = Units.inchesToMeters(0);
        public static final double targetHeight_Meters = Units.inchesToMeters(0);
        public static final double cameraPitch_Radians = Units.degreesToRadians(0);
    }
    public static final class AutoConstants{
        public static final double kPX = 0.1;
        public static final double kPY = 0.1;
        public static final double kPTheta = 0.1;
        public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(3, 2);
    }
}
