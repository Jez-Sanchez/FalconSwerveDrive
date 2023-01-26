// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignRobot extends CommandBase {
  private PIDController turn = new PIDController(0.03, 0, 12);
  private SwerveSubsystem swerve;
  private NetworkTable limelight;
  private double rotationSpeed;
  private boolean fieldRelative;
  private Translation2d translation = new Translation2d(0, 0);
  /** Creates a new AlignRobot. */
  public AlignRobot(SwerveSubsystem swerve, NetworkTable camera, boolean fieldRelative) {
    this.swerve = swerve;
    limelight = camera;
    this.fieldRelative = fieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double offsetX = limelight.getEntry("tx").getDouble(0.0);

    if(limelight.getEntry("tv").getDouble(0) == 1){
      //rotationSpeed = turn.calculate(result.getBestTarget().getYaw(), 0);
      rotationSpeed = turn.calculate(offsetX, 0);
      System.out.println("Rotation Speed: " + rotationSpeed);
    }
    // else{
    //   rotationSpeed = 0;
    // }
    swerve.drive(translation, rotationSpeed, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(rotationSpeed == 0){
      return true;
    }
    return false;
  }
}
