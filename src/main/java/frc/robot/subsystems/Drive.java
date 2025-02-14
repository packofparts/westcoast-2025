// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import poplib.src.main.java.poplib.control.PIDConfig;
import poplib.src.main.java.poplib.smart_dashboard.PIDTuning;

public class Drive extends SubsystemBase {
  private SparkMax leadRightMotor;
  private SparkMax followRightMotor;
  private SparkMax leadLeftMotor;
  private SparkMax followLeftMotor;
  private final DifferentialDrive drive;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private AHRS gyro;
  private static PIDController xpid;
  private static PIDController thetapid;
  private static double turnSetpoint;
  private static double Setpoint;
  protected PIDTuning leftTuning;
  protected PIDTuning rightTuning;
  private static Drive instance;

  public static Drive getInstance() {
    if (instance == null) {
        instance = new Drive();
      }
    return instance;
  }

  public Drive() {
    leadLeftMotor = DriveConstants.LeadLeftMotor.createSparkMax();
    followLeftMotor = DriveConstants.FollowLeftMotor.createSparkMax();
    leadRightMotor = DriveConstants.LeadRightMotor.createSparkMax();
    followRightMotor = DriveConstants.FollowRightMotor.createSparkMax();

    this.rightTuning = new PIDTuning("Right Motors", DriveConstants.LeadRightMotor.pid, DriveConstants.PID_TUNING_MODE);
    this.leftTuning = new PIDTuning("Left Motors", DriveConstants.LeadLeftMotor.pid, DriveConstants.PID_TUNING_MODE);
    
    kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
    
    gyro = new AHRS(null);
    gyro.reset();
    odometry = new DifferentialDriveOdometry
    (gyro.getRotation2d(), leadLeftMotor.getEncoder().getPosition(), 
    leadRightMotor.getEncoder().getPosition(), new Pose2d(0,0, new Rotation2d()));
    //.getPosition returns anmount of motor turns but we need distance traveled

    // xpid = DriveConstants.X_PID_CONFIG.getPIDController(); 
    // thetapid = DriveConstants.THETA_PID_CONFIG.getPIDController();

    turnSetpoint = 0.0;
    thetapid.setSetpoint(turnSetpoint);
    xpid.setSetpoint(Setpoint);
    
    drive = new DifferentialDrive(leadLeftMotor, leadRightMotor);
  }

  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(), leadLeftMotor.getEncoder().getPosition(), leadRightMotor.getEncoder().getPosition());
  }

  public Command turn(double angle){
    return runOnce(() -> {
      thetapid.setSetpoint(angle);
      thetapid.calculate(gyro.getAngle());
    }).andThen(run(() -> {
      leadLeftMotor.set(thetapid.calculate(gyro.getAngle())*DriveConstants.DRIVE_MOVE_SPEED);
    })).alongWith(run(() -> {
      leadRightMotor.set(thetapid.calculate(gyro.getAngle())*DriveConstants.DRIVE_REVERSE_SPEED);
    })).until(thetapid::atSetpoint).
    andThen(runOnce(() -> {
      leadLeftMotor.stopMotor();
      leadRightMotor.stopMotor();
    }));
  }

  public Command pleaseDrive(double velocity){
    return runOnce(() -> {
      xpid.setSetpoint(velocity);
      xpid.calculate(leadLeftMotor.get());
    }).andThen(run( () -> {
      leadLeftMotor.set(xpid.calculate(velocity));
    })).alongWith(run(() -> {
      leadRightMotor.set(xpid.calculate(velocity));
    }));
  }

  // public void driveArcade(double xSpeed, double zRotation) {
  //   drive.arcadeDrive(xSpeed, zRotation);
  // }

  @Override
  public void periodic() {
    updateOdometry();
    leftTuning.updatePID(leadLeftMotor);
    rightTuning.updatePID(leadRightMotor);
  }
}