// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivebase;


public class RobotContainer {

  private final Drivebase driveSubSystem = Drivebase.getDriveSubystem();
  
  //TODO: Make sure xbox connecting to port OperatorConstants.kDriverControllerPort = 0
  private final CommandXboxController driveController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }
  
  private void configureBindings() {
    Command cmd = new DriveCommand( () -> -driveController.getLeftY(),  () -> -driveController.getRightX(), driveSubSystem);
    driveSubSystem.setDefaultCommand(cmd);
  }
}
