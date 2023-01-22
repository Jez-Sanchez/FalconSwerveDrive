// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignRobot extends CommandBase {
  private PIDController turn = new PIDController(0.03, 0, 12);
  private SwerveSubsystem swerve;
  private PhotonCamera tracker;
  private double rotationSpeed;
  private boolean fieldRelative;
  private Translation2d translation = new Translation2d(0, 0);
  private SlewRateLimiter rotationLimit = new SlewRateLimiter(0.5);
  /** Creates a new AlignRobot. */
  public AlignRobot(SwerveSubsystem swerve, PhotonCamera camera, boolean fieldRelative) {
    this.swerve = swerve;
    tracker = camera;
    this.fieldRelative = fieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = tracker.getLatestResult();

    if(result.hasTargets()){
      //rotationSpeed = turn.calculate(result.getBestTarget().getYaw(), 0);
      rotationSpeed = rotationLimit.calculate(turn.calculate(result.getBestTarget().getYaw(), 0));
      System.out.println("Rotation Speed: " + rotationSpeed);
    }
    // else{
    //   rotationSpeed = 0;
    // }
    swerve.drive(translation, rotationSpeed, fieldRelative);
    if(result.getBestTarget().getYaw() == 0){
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
