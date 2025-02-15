// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private static double turnSetpoint;
  private static double Setpoint;
  protected PIDTuning leftTuning;
  protected PIDTuning rightTuning;
  private static Drivebase instance;
  private Translation2d offset;
  private Field2d field;
  private Pose2d currentPose;

  public static Drivebase getInstance() {
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
    
    gyro = new AHRS(null);
    gyro.reset();

    field = new Field2d();
    xpid.setTolerance(AutoAlign.X_TOLERANCE);
    thetapid.setTolerance(AutoAlign.THETA_TOLERANCE);

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

  public Command moveToPoseOdom(Supplier<Pose2d> poseSupplier, Translation2d newOffset) {
    return runOnce(() -> {
      offset = newOffset == null ? DriveConstants.AutoAlign.DEFAULT_OFFSET : newOffset;

      Pose2d pose = poseSupplier.get();
      Rotation2d targetRot = pose.getRotation();

      offset = offset.rotateBy(targetRot);
      Translation2d offsetTarget = pose.getTranslation().plus(offset); 

      field.getObject("target").setPose(new Pose2d(offsetTarget, targetRot));

      xpid.setSetpoint(offsetTarget.getX());
      thetapid.setSetpoint(targetRot.getRadians());

      xpid.calculate(currentPose.getX());
      thetapid.calculate(currentPose.getRotation().getRadians());
    }).andThen(run(
      () -> {
          turn(thetapid.calculate(currentPose.getRotation().getRadians()));
      }
  )).until(
    () -> thetapid.atSetpoint()
    ).andThen(run(
      () -> {
        pleaseDrive(xpid.calculate(currentPose.getX()));
      }
    )).until(
      () -> xpid.atSetpoint()
    ).andThen(
      () -> {
        xpid.close();
        thetapid.close();
      }
    );
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

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // leftTuning.updatePID(leadLeftMotor);
    // rightTuning.updatePID(leadRightMotor);

    Pose2d newodomPose = odometry.update(
        gyro.getRotation2d(),
  
        // TODO: Check if leadXXXMotor.getEncoder().getPosition() returns the distance in meters
        // TODO: Should we use both leadMotor and followerMotor encoders and than avarage them
        leadLeftMotor.getEncoder().getPosition(), 
        leadRightMotor.getEncoder().getPosition());
    this.field.setRobotPose(newodomPose);
  }
}