// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drive;
import poplib.src.main.java.poplib.controllers.oi.XboxOI;


public class RobotContainer {
  private Drive drive = new Drive();
  XboxOI oi;
  CommandXboxController driveController = new CommandXboxController(0);
  // private XboxOI driveJoystick;
  // private CommandJoystick transJoystick;

  public RobotContainer() {
    drive = Drive.getInstance();
    oi = XboxOI.getInstance();
    configureBindings();

    // driveJoystick = new XboxOI();
    // transJoystick = new CommandJoystick(1);
  }
  
  private void configureBindings() {
    // drive.setDefaultCommand(new DriveCommand(
    //   () -> -oi.getDriverTrigger(XboxController.Axis.kLeftY) *
    //       (oi.getDriverButton(XboxController.Button.kRightBumper) ? 1 : 0.5), 
    //   () -> -oi.getDriverTrigger(XboxController.Axis.kRightX),
    //   drive));

    drive.setDefaultCommand(
      new DriveCommand(() -> driveController.getLeftY(), () -> driveController.getRightX(), drive));

    oi.getDriverButton(XboxController.Axis.kLeftY.value).onChange(drive.pleaseDrive(XboxController.Axis.kLeftY.value));

    oi.getDriverButton(XboxController.Axis.kRightY.value).onChange(drive.turn(XboxController.Axis.kRightY.value));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
