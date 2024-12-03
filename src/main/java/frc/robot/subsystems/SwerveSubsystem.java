// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.FieldPosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
      SwerveConstants.leftFrontOffset
            );
    leftBack = new SwerveModule(
      SwerveConstants.leftBackTurning_ID,
      SwerveConstants.leftBackDrive_ID,
      SwerveConstants.leftBackAbsolutedEncoder_ID,
      SwerveConstants.leftBackTurningReverse,
      SwerveConstants.leftBackDriveReverse,
      SwerveConstants.leftBackOffset);
    rightFront = new SwerveModule(
      SwerveConstants.rightFrontTurning_ID,
      SwerveConstants.rightFrontDrive_ID,
      SwerveConstants.rightFrontAbsolutedEncoder_ID,
      SwerveConstants.rightFrontTurningReverse,
      SwerveConstants.rightFrontDriveReverse,
      SwerveConstants.rightFrontOffset);
    rightBack = new SwerveModule(
      SwerveConstants.rightBackTurning_ID,
      SwerveConstants.rightBackDrive_ID,
      SwerveConstants.rightBackAbsolutedEncoder_ID,
      SwerveConstants.rightBackTurningReverse,
      SwerveConstants.rightBackDriveReverse,
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

     // All other subsystem initialization
    // ...

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    // RobotConfig config;
    // try{
    //   config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    //   // Handle exception as needed
    //   e.printStackTrace();
    // }

    // // Configure AutoBuilder last
    // AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
    //         config, // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );

    // try{
    //   RobotConfig config = RobotConfig.fromGUISettings();

    //   // Configure AutoBuilder
    //   AutoBuilder.configure(
    //     this::getPose, 
    //     this::resetPose, 
    //     this::getSpeeds, 
    //     this::driveRobotRelative, 
    //     new PPHolonomicDriveController(
    //       Constants.Swerve.translationConstants,
    //       Constants.Swerve.rotationConstants
    //     ),
    //     config,
    //     () -> {
    //         // Boolean supplier that controls when the path will be mirrored for the red alliance
    //         // This will flip the path being followed to the red side of the field.
    //         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //         var alliance = DriverStation.getAlliance();
    //         if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //         }
    //         return false;
    //     },
    //     this
    //   );
    // }catch(Exception e){
    //   DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    // }

    // // Set up custom logging to add the current path to a field 2d widget
    // PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getRotation(), getModulesPosition());
    field.setRobotPose(odometry.getPoseMeters());
    
  }


  public ChassisSpeeds getChassisSpeed() {
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
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
    xSpeed = xSpeed * SwerveConstants.maxDriveSpeed_MeterPerSecond;
    ySpeed = ySpeed * SwerveConstants.maxDriveSpeed_MeterPerSecond;
    zSpeed = zSpeed * Math.toRadians(SwerveConstants.maxAngularVelocity_Angle);
    if(fieldOrient) {
      state = SwerveConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation()));//之後要處理MaxSpeedPerSecond跟MaxRadianPerSecond的問題
    }else{
      state = SwerveConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }
    setModouleStates(state);
  } 
}
