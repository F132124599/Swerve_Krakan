// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.FieldPosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveModule leftFront;
  private final SwerveModule leftBack;
  private final SwerveModule rightFront;
  private final SwerveModule rightBack;

  private final Pigeon2 gyro;
  private final Pigeon2Configuration gyroConfig;

  private final SwerveDriveOdometry odometry;

  private final Field2d field;


  public SwerveSubsystem() {
    leftFront = new SwerveModule(
      SwerveConstants.leftFrontTurning_ID,
      SwerveConstants.leftFrontDrive_ID,
      SwerveConstants.leftFrontAbsolutedEncoder_ID,
      SwerveConstants.leftFrontTurningReverse,
      SwerveConstants.leftFrontDriveReverse,
      SwerveConstants.leftFrontPid_Kp,
      SwerveConstants.leftFrontPid_Ki,
      SwerveConstants.leftFrontPid_Kd,
      SwerveConstants.leftFrontOffset
            );
    leftBack = new SwerveModule(
      SwerveConstants.leftBackTurning_ID,
      SwerveConstants.leftBackDrive_ID,
      SwerveConstants.leftBackAbsolutedEncoder_ID,
      SwerveConstants.leftBackTurningReverse,
      SwerveConstants.leftBackDriveReverse,
      SwerveConstants.leftbackPid_Kp,
      SwerveConstants.leftBackPid_Ki,
      SwerveConstants.leftBackPid_Kd,
      SwerveConstants.leftBackOffset);
    rightFront = new SwerveModule(
      SwerveConstants.rightFrontTurning_ID,
      SwerveConstants.rightFrontDrive_ID,
      SwerveConstants.rightFrontAbsolutedEncoder_ID,
      SwerveConstants.rightFrontTurningReverse,
      SwerveConstants.rightFrontDriveReverse,
      SwerveConstants.rightFrontPid_Kp,
      SwerveConstants.rightFrontPid_Ki,
      SwerveConstants.rightFrontPid_Kd,
      SwerveConstants.rightFrontOffset);
    rightBack = new SwerveModule(
      SwerveConstants.rightBackTurning_ID,
      SwerveConstants.rightBackDrive_ID,
      SwerveConstants.rightBackAbsolutedEncoder_ID,
      SwerveConstants.rightBackTurningReverse,
      SwerveConstants.rightBackDriveReverse,
      SwerveConstants.rightBackPid_Kp,
      SwerveConstants.rightBackPid_Ki,
      SwerveConstants.rightBackPid_Kd,
      SwerveConstants.rightBackOffset);

     gyro = new Pigeon2(SwerveConstants.gyro_ID);
     gyroConfig = new Pigeon2Configuration();

     gyroConfig.MountPose.MountPoseYaw = 0;
     gyroConfig.MountPose.MountPosePitch = 0;
     gyroConfig.MountPose.MountPoseRoll = 0;

     gyro.getConfigurator().apply(gyroConfig);

     field = new Field2d();

     odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getRotation(), getModulesPosition(), getRobotPose());

     resetGyro();
  }


  public Pose2d getRobotPose() {
    return field.getRobotPose();
  }

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }

  public SwerveModulePosition[] getModulesPosition() {
    return new SwerveModulePosition[]{
      leftFront.getPosition(),
      leftBack.getPosition(),
      rightFront.getPosition(),
      rightBack.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      leftFront.getState(),
      leftBack.getState(),
      rightFront.getState(),
      rightBack.getState()
    };
  }

  public void setModouleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxDriveSpeed_MeterPerSecond);
      leftFront.setState(desiredStates[0]);
      leftBack.setState(desiredStates[1]);
      rightFront.setState(desiredStates[2]);
      rightBack.setState(desiredStates[3]);
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void setPose(Pose2d poses) {
    odometry.resetPosition(getRotation(), getModulesPosition(), poses);
  }


  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient) {
    SwerveModuleState[] state;
    if(fieldOrient) {
      state = SwerveConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation()));//之後要處理MaxSpeedPerSecond跟MaxRadianPerSecond的問題
    }else{
      state = SwerveConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }
    setModouleStates(state);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getRotation(), getModulesPosition());
  }
}
