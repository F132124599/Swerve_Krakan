// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final com.ctre.phoenix6.hardware.TalonFX turningMotor;
  private final com.ctre.phoenix6.hardware.TalonFX driveMotor;

  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration cancoderConfig;

  private final PIDController turningPidController;
  private final PIDController drivePidController;
  private final SimpleMotorFeedforward driveFeedForward;

  private double turningPidOutput;
  private double drivePidOutput;
  private double driveFeedForwardPutput;
  private double driveOutput;


  public SwerveModule(int turningMotor_ID, int driveMotor_ID, int absolutedEncoder_ID, boolean turningMotorReverse, boolean driveMotorReverse, double offset) {
    turningMotor = new com.ctre.phoenix6.hardware.TalonFX(turningMotor_ID);
    driveMotor = new com.ctre.phoenix6.hardware.TalonFX(driveMotor_ID);

    absolutedEncoder = new CANcoder(absolutedEncoder_ID);
    cancoderConfig = new CANcoderConfiguration();

    turningPidController = new PIDController(ModuleConstants.turningPidController_Kp, ModuleConstants.turningPidController_Ki, ModuleConstants.turningPidController_Kd);
    turningPidController.enableContinuousInput(ModuleConstants.pidRangeMin, ModuleConstants.pidRangeMax);

    drivePidController = new PIDController(driveMotor_ID, absolutedEncoder_ID, offset);
    driveFeedForward = new SimpleMotorFeedforward(absolutedEncoder_ID, offset);

    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.MagnetOffset = offset;

    absolutedEncoder.getConfigurator().apply(cancoderConfig);

    turningMotor.setInverted(turningMotorReverse);
    driveMotor.setInverted(driveMotorReverse);

    turningMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);

    resetEncoder();
  }

  public void resetEncoder() {
    driveMotor.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningAngle()));
  }

  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble()*ModuleConstants.driveEncoderRot2MeterPerSec;
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble()*ModuleConstants.driveEncoderRot2MeterPerSec;
  }

  public double getTurningPosition() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getTurningAngle() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public void stopMotor() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void setState(SwerveModuleState state) {
    // Turn Motor
      SwerveModuleState optimizedState = SwerveModuleState.optimize(state,getState().angle);
      double turningMotorOutput = turningPidController.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());
      turningMotor.set(turningMotorOutput);
    // Drive motor
      double driveFeedforwardOutPut = driveFeedForward.calculate(optimizedState.speedMetersPerSecond)/12;
      double drivePidOutput = drivePidController.calculate(getDriveVelocity() / SwerveConstants.driveEncoderRPM2MeterPerSec, optimizedState.speedMetersPerSecond / SwerveConstants.driveEncoderRPM2MeterPerSec);
      double driveMotorOutput = driveFeedforwardOutPut + drivePidOutput;
      this.driveFeedForwardPutput = driveFeedforwardOutPut;
      this.drivePidOutput = drivePidOutput;
      this.driveOutput = driveMotorOutput;
      driveMotor.set(driveMotorOutput);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
