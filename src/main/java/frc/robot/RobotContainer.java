// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {

  private final Drivebase driveSubsystem = Drivebase.getDriveSubystem();
  private final XboxController driveController = new XboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }
  
  private double xSpeed() {
    return -driveController.getLeftY();
  }

  private void configureBindings() {
    Command cmd = new DriveCommand(this::xSpeed, ()-> -driveController.getRightX(), driveSubsystem);
    driveSubsystem.setDefaultCommand(cmd);
  }
}
