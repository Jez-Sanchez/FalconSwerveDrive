// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.chassisConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class SwerveJoystickCmd extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean overrideJS;
  
  private SwerveSubsystem swerve;
  private XboxController driver;
  private SlewRateLimiter yLim = new SlewRateLimiter(1);
  private SlewRateLimiter xLim = new SlewRateLimiter(1);
  private SlewRateLimiter rotLim = new SlewRateLimiter(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveJoystickCmd(SwerveSubsystem swerve, XboxController driver, boolean fieldRelative) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    this.driver = driver;
    this.fieldRelative = fieldRelative;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yAxis = -driver.getLeftY();
    double xAxis = -driver.getLeftX();
    double rotAxis = -driver.getRightX();
    yAxis = (Math.abs(yAxis) < chassisConstants.deadband ? 0 : yLim.calculate(yAxis * 0.3));
    xAxis = (Math.abs(xAxis) < chassisConstants.deadband ? 0 : xLim.calculate(xAxis * 0.3));
    rotAxis = (Math.abs(rotAxis) < chassisConstants.deadband ? 0 : rotLim.calculate(rotAxis * 0.3));

    translation = new Translation2d(yAxis, xAxis).times(chassisConstants.maxSpeedMPS);
    rotation = rotAxis * chassisConstants.maxTurnSpeed;
    swerve.drive(translation, rotAxis, fieldRelative);
    // if(xAxis == 0 && yAxis == 0 && rotAxis == 0){
    //   swerve.zeroModules();
    //   swerve.stopModules();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
