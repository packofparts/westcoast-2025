// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.AutoAlign;
import poplib.smart_dashboard.PIDTuning;

public class Drivebase extends SubsystemBase {
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
  protected PIDTuning leftTuning;
  protected PIDTuning rightTuning;
  private static Drivebase instance;
  private Pose2d currentPose;
  private double X;
  private double Y;
  private double theta;


  public static Drivebase getDriveSubystem() {
    if (instance == null) {
        instance = new Drivebase();
      }
    return instance;
  }

  public Drivebase() {
    leadLeftMotor = DriveConstants.LeadLeftMotor.createSparkMax();
    followLeftMotor = DriveConstants.FollowLeftMotor.createSparkMax();
    leadRightMotor = DriveConstants.LeadRightMotor.createSparkMax();
    followRightMotor = DriveConstants.FollowRightMotor.createSparkMax();

    this.rightTuning = new PIDTuning("Right Motors", DriveConstants.LeadRightMotor.pid, DriveConstants.PID_TUNING_MODE);
    this.leftTuning = new PIDTuning("Left Motors", DriveConstants.LeadLeftMotor.pid, DriveConstants.PID_TUNING_MODE);
    
    kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
    
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.reset();

    xpid.setTolerance(AutoAlign.X_TOLERANCE);
    thetapid.setTolerance(AutoAlign.THETA_TOLERANCE);
    currentPose = new Pose2d();

    odometry = new DifferentialDriveOdometry
    (gyro.getRotation2d(), leadLeftMotor.getEncoder().getPosition(), 
    leadRightMotor.getEncoder().getPosition(), new Pose2d(0,0, new Rotation2d()));

    xpid = DriveConstants.X_PID_CONTROLLER;
    thetapid = DriveConstants.THETA_PID_CONTROLLER;
    drive = new DifferentialDrive(leadLeftMotor, leadRightMotor);
  }

  public Command moveToPose(Pose2d targetPose2d) {
     return runOnce(() -> {
      X = Math.abs(currentPose.getX() - targetPose2d.getX());
      Y = Math.abs(currentPose.getY() - targetPose2d.getY());
      theta = Math.atan(Y/X);
     }).andThen(
      turnToPose(theta)
     ).andThen(
      driveToPose(targetPose2d)
     );
  }

  public Command turnToPose(double angle){
    return runOnce(() -> {
      thetapid.setSetpoint(0);
      thetapid.calculate(gyro.getAngle());
    }).andThen(run(() -> {
      leadLeftMotor.set(thetapid.calculate(gyro.getAngle())*DriveConstants.DRIVE_MOVE_SPEED);
    })).alongWith(run(() -> {
      leadRightMotor.set(thetapid.calculate(gyro.getAngle())*DriveConstants.DRIVE_REVERSE_SPEED);
    })).until(thetapid::atSetpoint).
    andThen(runOnce(() -> {
      leadLeftMotor.stopMotor();
      leadRightMotor.stopMotor();
      thetapid.close();
    }));
  }

  public Command driveToPose(Pose2d targetPose){
    return runOnce(() -> {
      xpid.setSetpoint(0.0);
      X = Math.abs(currentPose.getX() - targetPose.getX());
      Y = Math.abs(currentPose.getY() - targetPose.getY());
      xpid.calculate(Math.hypot(X, Y));
    }).andThen(run( () -> {
      X = Math.abs(currentPose.getX() - targetPose.getX());
      Y = Math.abs(currentPose.getY() - targetPose.getY());
      leadLeftMotor.set(xpid.calculate(Math.hypot(X, Y)));
      leadRightMotor.set(xpid.calculate(Math.hypot(X, Y)));
    })).until(xpid::atSetpoint)
    .andThen(runOnce(() -> {
      xpid.close();
      leadLeftMotor.stopMotor();
      leadRightMotor.stopMotor();
    }));
  }

  public Command moveForward(){
    return run(()->{
      leadLeftMotor.set(-1);
      leadRightMotor.set(-1);
    });
  }

  public Command reverseCommand(){
    return run(()-> {
      leadLeftMotor.set(Constants.DriveConstants.DRIVE_REVERSE_SPEED);
      leadRightMotor.set(Constants.DriveConstants.DRIVE_REVERSE_SPEED);
    });
  }
    
  public Command stopCommand(){
    return runOnce(() -> {
      leadLeftMotor.set(Constants.DriveConstants.STOP_SPEED);
      leadRightMotor.set(Constants.DriveConstants.STOP_SPEED);
    });
  }

  public void updateOdometry() {
    double leftEncoderValue = leadLeftMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Left encoder value", leftEncoderValue);

    double rightEncoderValue = leadRightMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Right encoder value", rightEncoderValue);

    odometry.update(gyro.getRotation2d(), leftEncoderValue, rightEncoderValue);
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    leftTuning.updatePID(leadLeftMotor);
    rightTuning.updatePID(leadRightMotor);
    updateOdometry();
    Pose2d pose = odometry.getPoseMeters();
    SmartDashboard.putNumber("Robot pose: x", pose.getX());
    SmartDashboard.putNumber("Robot pose: y", pose.getY());
    SmartDashboard.putString("Robot pose: heading", pose.getRotation().toString());
  }
}