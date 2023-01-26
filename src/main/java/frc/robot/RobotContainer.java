// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.chassisConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController driver = new XboxController(0);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //private final SwerveJoystickCmd m_autoCommand = new SwerveJoystickCmd(swerveSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem, driver, fieldRelative));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    PathPlannerTrajectory path1 = PathPlanner.loadPath("PathGroup1", new PathConstraints(4, 3));
    List<PathPlannerTrajectory> pathGroup1 = PathPlanner.loadPathGroup("PathGroup1", 3, 2);
    return new SequentialCommandGroup(new InstantCommand(() -> swerveSubsystem.resetOdometry(path1.getInitialHolonomicPose())),
    new PPSwerveControllerCommand(path1, swerveSubsystem::getPose, chassisConstants.swerveKinematics, 
    new PIDController(0.3, 0, 0), new PIDController(0.3, 0, 0), 
    new PIDController(0.3, 0, 0), swerveSubsystem::setModuleStates, swerveSubsystem),
    new InstantCommand(() -> swerveSubsystem.stopModules()));
    //return m_autoCommand;
  }
}