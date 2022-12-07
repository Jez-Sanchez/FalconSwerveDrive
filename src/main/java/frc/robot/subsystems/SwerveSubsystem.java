// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.chassisConstants;
import frc.robot.Constants.chassisSetUp;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule frontLeft = new SwerveModule(chassisSetUp.fLeftDriveMotorPort, chassisSetUp.isFrontLeftDriveMotorReverse, 
  chassisSetUp.fLeftTurnMotorPort, chassisSetUp.isFrontLeftTurnMotorReverse, chassisSetUp.fLeftAbsoluteEncoder, chassisSetUp.frontLAngle,chassisSetUp.frontLKP, chassisSetUp.frontLKI, chassisSetUp.frontLKD);

  private final SwerveModule frontRight = new SwerveModule(chassisSetUp.fRightDriveMotorPort, chassisSetUp.isFrontRightDriveMotorReverse, 
  chassisSetUp.fRightTurnMotorPort, chassisSetUp.isFrontRightTurnMotorReverse, chassisSetUp.fRightAbsoluteEncoder, chassisSetUp.frontRAngle, chassisSetUp.frontRKP, chassisSetUp.frontRKI, chassisSetUp.frontRKD);

  private final SwerveModule backLeft = new SwerveModule(chassisSetUp.bLeftDriveMotorPort, chassisSetUp.isBackLeftDriveMotorReverse, 
  chassisSetUp.bLeftTurnMotorPort, chassisSetUp.isBackLeftTurnMotorReverse, chassisSetUp.bLeftAbsoluteEncoder, chassisSetUp.backLAngle, chassisSetUp.backLKP, chassisSetUp.backLKI, chassisSetUp.backLKD);

  private final SwerveModule backRight = new SwerveModule(chassisSetUp.bRightDriveMotorPort, chassisSetUp.isBackRightDriveMotorReverse, 
  chassisSetUp.bRightTurnMotorPort, chassisSetUp.isBackRightTurnMotorReverse, chassisSetUp.bRightAbsoluteEncoder, chassisSetUp.backRAngle, chassisSetUp.backRKP, chassisSetUp.backRKI, chassisSetUp.backRKD);

  private WPI_Pigeon2 gyro = new WPI_Pigeon2(0);
  SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(chassisConstants.swerveKinematics, getYaw());

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
    SmartDashboard.putNumber("FrontL Angle: ", frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("FrontR Angle: ", frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("BackL Angle: ", backLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("BackR Angle: ", backRight.getState().angle.getDegrees());

  }
  // public void stopModules(){
  //   frontLeft.stop();
  //   frontRight.stop();
  //   backLeft.stop();
  //   backRight.stop();
  // }
  
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        Constants.chassisConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation, 
                                getYaw()
                            )
                            : new ChassisSpeeds(
                                translation.getX(), 
                                translation.getY(), 
                                rotation)
                            );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, chassisConstants.maxSpeedMPS);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }    
  
  public Rotation2d getYaw() {
    double[] ypr = new double[3];
    gyro.getYawPitchRoll(ypr);
    return (chassisSetUp.invertedGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
}
}