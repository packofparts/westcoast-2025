// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import poplib.motor.MotorConfig;
import edu.wpi.first.math.controller.PIDController;
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

    public static final boolean PID_TUNING_MODE = false;

    public static final int DRIVE_MOVE_SPEED = 1;
    public static final int DRIVE_REVERSE_SPEED = -1;
    public static final double TRACK_WIDTH = 0;
    public static final double WHEEL_RADIUS = 0;
    public static final double ENCODER_RESOLUTION = 0;
    public static final PIDController X_PID_CONTROLLER = new PIDConfig(0, 0, 0, 0).getPIDController();
    public static final PIDController THETA_PID_CONTROLLER = new PIDConfig(0, 0, 0, 0).getPIDController();

    // TODO: Make sure the CAN IDs are correct for 4 motors 
    // TODO: Make sure the current limit is 20A or 60A
    // MotorConfig(int canId, int currentLimit, Boolean inversion, Mode mode)
    public static final MotorConfig LeadLeftMotor = new MotorConfig(
      1,
      20, 
      true, 
      Mode.COAST
    );

    // FollowerConfig(MotorConfig leadConfig, boolean inverted, int canId)
    public static final FollowerConfig FollowLeftMotor = new FollowerConfig(
      LeadLeftMotor,
      false,
      2
    );

    // MotorConfig(int canId, int currentLimit, Boolean inversion, Mode mode)
    public static final MotorConfig LeadRightMotor = new MotorConfig(
      3,
      20, 
      false, 
      Mode.COAST
    );

    // FollowerConfig(MotorConfig leadConfig, boolean inverted, int canId)
    public static final FollowerConfig FollowRightMotor = new FollowerConfig(
      LeadRightMotor,
      false,
      4
    );
  }
}
