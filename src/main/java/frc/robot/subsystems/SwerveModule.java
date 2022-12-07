// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.chassisConstants;
import frc.robot.Constants.chassisSetUp;

public class SwerveModule extends SubsystemBase {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  // private final TalonFXSensorCollection driveMotorEnc;
  // private final TalonFXSensorCollection turnMotorEnc

  //private DigitalInput absoluteEnc;
  private DutyCycleEncoder absoluteEncoder;
  private double absoluteEncoderOffset;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.chassisConstants.kS, Constants.chassisConstants.kV, Constants.chassisConstants.kA); //Revise and 
  /** Creates a new ExampleSubsystem. */
  public SwerveModule(int driveMotorID, boolean driveMotorReversed, int turnMotorID, boolean turnMotorReversed, int absoluteEncoderID, double offset, double kP, double kI, double kD) {
    this.driveMotor = new TalonFX(driveMotorID);
    this.turnMotor = new TalonFX(turnMotorID);

    driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
    turnMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    TalonFXSensorCollection sensorCollection = turnMotor.getSensorCollection();
    double absoluteValue = sensorCollection.getIntegratedSensorAbsolutePosition();
    

    // driveMotorEnc = new TalonFXSensorCollection(driveMotor);
    // turnMotorEnc = new TalonFXSensorCollection(turnMotor);

    //turnMotor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

    driveMotor.setNeutralMode(NeutralMode.Brake);
    turnMotor.setNeutralMode(NeutralMode.Brake);

    // turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    driveMotor.setInverted(driveMotorReversed);
    turnMotor.setInverted(turnMotorReversed);

    //absoluteEnc = new DigitalInput(absoluteEncoderID);

    absoluteEncoder = new DutyCycleEncoder(absoluteEncoderID);
    absoluteEncoder.setConnectedFrequencyThreshold(1);
    this.absoluteEncoderOffset = offset;

    absoluteEncoder.setDutyCycleRange(0, 1);

    turnMotor.config_kP(0, kP);
    turnMotor.config_kI(0, kI);
    turnMotor.config_kD(0, kD);

    //resetEncoder();
  }

  // public void resetEncoder(){
  //   driveMotor.setSelectedSensorPosition(0);
  //   absoluteEncoder.reset();
  //   turnMotor.setSelectedSensorPosition(degreesToFalcons(getTurnAngle().getDegrees(), chassisConstants.turnMotorGearRat));
  // }
  public Rotation2d getTurnAngle(){
    return new Rotation2d((2 * Math.PI/(2048 * chassisConstants.turnMotorGearRat)) * (turnMotor.getSelectedSensorPosition() % (2048 * chassisConstants.turnMotorGearRat)));
  }
  public double falconToDegrees(double counts, double gearRatio){
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  public double degreesToFalcons(double degrees, double gearRatio){
    double ticks = degrees / (360.0 / (gearRatio * 2048.0));
    return ticks;
  }

  public double falconToRPM(double velocityCounts, double gearRatio){
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }

  public double RPMToFalcon(double RPM, double gearRatio){
    double motorRPM = RPM * gearRatio;
    double sensorCounts = motorRPM * (2048.0 / 600.0);
    return sensorCounts;
  }

  public double falconToMPS(double velocitycounts, double circumference, double gearRatio){
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  public double MPSToFalcons(double velocity, double circumference, double gearRatio){
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }

  public SwerveModuleState getState(){
    double velocity = falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.chassisConstants.circumference, Constants.chassisConstants.turnMotorGearRat);
    Rotation2d angle = getTurnAngle();
    return new SwerveModuleState(velocity, angle);
  }

  public void setDesiredState(SwerveModuleState desiredState){
    desiredState = SwerveModuleState.optimize(desiredState, getTurnAngle());

    double velocity = MPSToFalcons(desiredState.speedMetersPerSecond, Constants.chassisConstants.circumference, Constants.chassisConstants.driveMotorGearRat);
    //double turnOutput = turnPIDController.calculate(turnAngleRadians(), desiredState.angle.getRadians());
    driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    long nearestDegree = Math.round(desiredState.angle.getDegrees());

    double setTurnValue = (2048 / 360) * nearestDegree;
    double inputAngle = nearestDegree;
    double setPoint = setTurnValue * chassisConstants.turnMotorGearRat;
    turnMotor.set(ControlMode.Position, setPoint);
    System.out.println("Turn Motor Position: " + degreesToFalcons(90, chassisConstants.turnMotorGearRat));
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
