// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import poplib.motor.MotorConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import poplib.control.PIDConfig;
import poplib.motor.FollowerConfig;
import poplib.motor.Mode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {

    public static class AutoAlign{
      public static final Translation2d DEFAULT_OFFSET = new Translation2d(0.5, 0.0);
      
      public static final double X_TOLERANCE = 0.1;
      public static final double THETA_TOLERANCE = edu.wpi.first.math.util.Units.degreesToRadians(2.0);
    } 

    public static final boolean PID_TUNING_MODE = false;

    public static final int DRIVE_MOVE_SPEED = -1;
    public static final int DRIVE_REVERSE_SPEED = 1;
    public static final int STOP_SPEED = 0;
    public static final double TRACK_WIDTH = 0;
    public static final double WHEEL_RADIUS = 0;
    public static final double ENCODER_RESOLUTION = 0;
    public static final PIDController X_PID_CONTROLLER = new PIDConfig(0, 0, 0, 0).getPIDController();
    public static final PIDController THETA_PID_CONTROLLER = new PIDConfig(0, 0, 0, 0).getPIDController();

    public static final MotorConfig LeadLeftMotor = new MotorConfig(
      2,
      40, 
      true, 
      Mode.COAST
    );

    public static final FollowerConfig FollowLeftMotor = new FollowerConfig(
      LeadLeftMotor,
      false,
      5
    );

    public static final MotorConfig LeadRightMotor = new MotorConfig(
      4,
      40, 
      false, 
      Mode.COAST
    );

    public static final FollowerConfig FollowRightMotor = new FollowerConfig(
      LeadRightMotor,
      false,
      3
    );
  }
}