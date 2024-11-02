// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

  public static class ModuleConstants {

    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/6.75;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2Rad = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;

    public static final double kModuleDistance = 21*0.0254;

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );



  }

  public class SwerveConstants {
    public static final int leftFrontDrive_ID = 0;
    public static final int leftBackDrive_ID = 0;
    public static final int rightFrontDrive_ID = 0;
    public static final int rightBackDrive_ID = 0;

    public static final int leftFrontTurning_ID = 0;
    public static final int leftBackTurning_ID = 0;
    public static final int rightFrontTurning_ID = 0;
    public static final int rightBackTurning_ID = 0;

    public static final int leftFrontAbsolutedEncoder_ID = 0;
    public static final int leftBackAbsolutedEncoder_ID = 0;
    public static final int rightFrontAbsolutedEncoder_ID = 0;
    public static final int rightBackAbsolutedEncoder_ID = 0;

    public static final double leftFrontOffset = 0;
    public static final double leftBackOffset = 0;
    public static final double rightFrontOffset = 0;
    public static final double rightBackOffset = 0;

    public static final double leftFrontPid_Kp = 0;
    public static final double leftbackPid_Kp = 0;
    public static final double rightFrontPid_Kp = 0;
    public static final double rightBackPid_Kp = 0;

    public static final double leftFrontPid_Ki = 0;
    public static final double leftBackPid_Ki = 0;
    public static final double rightFrontPid_Ki = 0;
    public static final double rightBackPid_Ki = 0;

    public static final double leftFrontPid_Kd = 0;
    public static final double leftBackPid_Kd = 0;
    public static final double rightFrontPid_Kd = 0;
    public static final double rightBackPid_Kd = 0;

    public static final boolean leftFrontTurningReverse = false;
    public static final boolean leftBackTurningReverse = false;
    public static final boolean rightFrontTurningReverse = false;
    public static final boolean rightBackTurningReverse = false;

    public static final boolean leftFrontDriveReverse = false;
    public static final boolean leftBackDriveReverse = false;
    public static final boolean rightFrontDriveReverse = false;
    public static final boolean rightBackDriveReverse = false;

    public static final int gyro_ID = 0;


    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/6.75;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2Rad = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;

    public static final double kModuleDistance = 21*0.0254;


    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2));

    
    public static final double pathingMoving_Kp = 12;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0.056;

    public static final double pathingtheta_Kp = 3;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0.035;

    public static final double maxOutput = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 5;
    public static final double kDriveBaseRadius = 14.85 * 0.0254;

    
  }
}
