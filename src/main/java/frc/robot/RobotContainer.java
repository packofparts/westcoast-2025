// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drivebase;
import poplib.controllers.oi.OI;
import poplib.controllers.oi.XboxOI;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {

  private final Drivebase driveSubsystem = Drivebase.getDriveSubystem();
  private final XboxController driveController = new XboxController(0);
  public final OI oi;
  
  //TODO: Make sure xbox connecting to port OperatorConstants.kDriverControllerPort = 0

  public RobotContainer() {
    oi = new XboxOI();
    configureBindings();
  }
  
  private void configureBindings() {
    Command cmd = new DriveCommand(() -> -driveController.getLeftY() , ()-> -driveController.getRightX(), driveSubsystem);
    driveSubsystem.setDefaultCommand(cmd);
    // oi.getDriverButton(XboxController.Button.kA.value).onTrue(driveSubSystem.moveForward()).onFalse(driveSubSystem.stopCommand());
  }
}
