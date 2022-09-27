// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX; 
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX driveMotor;
  private final TalonFX turningMotor;

  // private final CANCoder driveEncoder;
  // private final CANCoder turningEncoder;

  private final PIDController turningPidController;

  private DutyCycleEncoder absoluteEncoder;
  private final boolean absoluteEncoderReversed;
  private final double absoluteEncoderOffsetRad;
  public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, 
          int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
            this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            absoluteEncoder = new DutyCycleEncoder(absoluteEncoderId);

            driveMotor = new TalonFX(driveMotorID);
            turningMotor = new TalonFX(turningMotorID);

            driveMotor.setInverted(driveMotorReversed);
            turningMotor.setInverted(turningMotorReversed);

            turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
            turningPidController.enableContinuousInput(-Math.PI, Math.PI);

            resetEncoders();
  }
  

  public double getDrivePosition(){
    System.out.println("This is the Sensor Driving Position" + driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter;//checks to see if it works when testing the chassis
  }

  public double getTurningPosition(){
    System.out.println("This is the Sensor Turning Position" + turningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    return turningMotor.getSelectedSensorPosition();//check to see if it works when testing the chassis
  }

  public double getDriveVelocity(){
    return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;//check to see if it works when testing the chassis
  }

  public double getTurningVelocity(){
    return turningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderRPM2RadPerSec;//check to see if it works when testing the chassis
  }
  public double getAbsoluteEncoderRad(){
    // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    // angle *= 2.0 * Math.PI;
    // angle -= absoluteEncoderOffsetRad;
    // System.out.println(angle);
    // return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    double angle = absoluteEncoder.getAbsolutePosition();
    System.out.println("Absolute Encoder Position " + absoluteEncoder.getAbsolutePosition());
    return angle;
  }

  public void resetEncoders(){
    driveMotor.setSelectedSensorPosition(0);
    turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  public void setDesiredState(SwerveModuleState state){
    state = SwerveModuleState.optimize(state, getState().angle);
    if(Math.abs(state.speedMetersPerSecond) < 0.001){
      stop();
      return;
    }
    double falconSpeed = Units.feetToMeters(17.01);
    driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    System.out.println("Drive Motor Output " + driveMotor.getMotorOutputPercent());
    turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    System.out.println(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getSourceChannel() + "] state", state.toString()); 
  }
  public void stop(){
    driveMotor.set(ControlMode.PercentOutput, 0);
    turningMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
