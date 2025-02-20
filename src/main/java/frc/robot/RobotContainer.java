// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveRectangleCommand;
import frc.robot.subsystems.Drivebase;
import poplib.controllers.oi.XboxOI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {
  private final Drivebase driveSubsystem = new Drivebase();
  private final XboxController driveController = new XboxController(0);
  private final XboxOI oi;
   private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    oi = new XboxOI();
    configureBindings();
    autoChooser.setDefaultOption("Drive Rectangle", new DriveRectangleCommand());
  }

  private double xSpeed() {
    return driveController.getLeftY();
  }

  private void configureBindings() {
    // SequentialCommandGroup cmd = new DriveRectangleCommand();
    Command cmd = new DriveCommand(this::xSpeed, ()-> -driveController.getRightX(), driveSubsystem);
    driveSubsystem.setDefaultCommand(cmd);
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
