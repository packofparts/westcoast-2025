// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drivebase extends SubsystemBase {
  private final SparkMax leadRightMotor;
  private final SparkMax followRightMotor;
  private final SparkMax leadLeftMotor;
  private final SparkMax followLeftMotor;

  private final DifferentialDrive differentialDrive;
  private final DifferentialDriveOdometry odometry;

  // TODO: Check this with hardware person about the gyro sensor com type
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  
  private static Drivebase driveSubystem;
  public static Drivebase getDriveSubystem() {
    if (driveSubystem == null) {
        driveSubystem = new Drivebase();
      }
    return driveSubystem;
  }

  public Drivebase() {
    leadLeftMotor = DriveConstants.LeadLeftMotor.createSparkMax();
    followLeftMotor = DriveConstants.FollowLeftMotor.createSparkMax();
    leadRightMotor = DriveConstants.LeadRightMotor.createSparkMax();
    followRightMotor = DriveConstants.FollowRightMotor.createSparkMax();    
    differentialDrive = new DifferentialDrive(leadLeftMotor, leadRightMotor);    

    gyro.reset();

    // TODO: Make sure when the Drivebase is created, the initialRotation, leftDistanceMeters, rightDistanceMeters are setting properly
    Rotation2d initialRotation = gyro.getRotation2d();
    double leftDistanceMeters = 0;
    double rightDistanceMeters = 0;
    odometry = new DifferentialDriveOdometry(initialRotation, leftDistanceMeters, rightDistanceMeters);
    SmartDashboard.putString("odometry started at: ", initialRotation.toString() + ", leftDistanceMeters: " + leftDistanceMeters + ", rightDistanceMeters = " + rightDistanceMeters);
  }

  public void updateOdometry() {
    double leftEncoderValue = leadLeftMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Left encoder value", leftEncoderValue);

    double rightEncoderValue = leadRightMotor.getEncoder().getPosition();
    SmartDashboard.putNumber("Right encoder value", rightEncoderValue);

    odometry.update(gyro.getRotation2d(), leftEncoderValue, rightEncoderValue);
  }

  public void driveArcade(double xSpeed, double zRotation) {
    differentialDrive.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    updateOdometry();
    Pose2d pose = odometry.getPoseMeters();
    SmartDashboard.putNumber("Robot pose: x", pose.getX());
    SmartDashboard.putNumber("Robot pose: y", pose.getY());
    SmartDashboard.putString("Robot pose: heading", pose.getRotation().toString());
  }
}