// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.kFrontLeftDriveMotorPort, DriveConstants.kFrontLeftTurningMotorPort, DriveConstants.kFrontLeftDriveEncoderReversed, 
  DriveConstants.kFrontLeftTurningEncoderReversed, DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
  DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule frontRight = new SwerveModule(DriveConstants.kFrontRightDriveMotorPort, DriveConstants.kFrontRightTurningMotorPort, DriveConstants.kFrontRightDriveEncoderReversed, 
  DriveConstants.kFrontRightTurningEncoderReversed, DriveConstants.kFrontRightDriveAbsoluteEncoderPort, DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
  DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

  private final SwerveModule backLeft = new SwerveModule(DriveConstants.kBackLeftDriveMotorPort, DriveConstants.kBackLeftTurningMotorPort, DriveConstants.kBackLeftDriveEncoderReversed, 
  DriveConstants.kBackLeftTurningEncoderReversed, DriveConstants.kBackLeftDriveAbsoluteEncoderPort, DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
  DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

  private final SwerveModule backRight = new SwerveModule(DriveConstants.kBackRightDriveMotorPort, DriveConstants.kBackRightTurningMotorPort, DriveConstants.kBackRightDriveEncoderReversed, 
  DriveConstants.kBackRightTurningEncoderReversed, DriveConstants.kBackRightDriveAbsoluteEncoderPort, DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
  DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

  private WPI_Pigeon2 gyro = new WPI_Pigeon2(0);

  public SwerveSubsystem() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch(Exception e){
      }
    }).start();
  }
  public void zeroHeading(){
    gyro.reset();
  }

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }
  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading", getHeading());
  }
  public void stopModules(){
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
