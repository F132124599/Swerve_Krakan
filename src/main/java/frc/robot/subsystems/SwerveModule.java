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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final com.ctre.phoenix6.hardware.TalonFX turningMotor;
  private final com.ctre.phoenix6.hardware.TalonFX driveMotor;

  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration cancoderConfig;

  private final PIDController turningPidController;

  private double turningPidOutput;


  public SwerveModule(int turningMotor_ID, int driveMotor_ID, int absolutedEncoder_ID, boolean turningMotorReverse, boolean driveMotorReverse, double pid_Kp, double pid_Ki, double pid_Kd, double offset) {
    turningMotor = new com.ctre.phoenix6.hardware.TalonFX(turningMotor_ID);
    driveMotor = new com.ctre.phoenix6.hardware.TalonFX(driveMotor_ID);

    absolutedEncoder = new CANcoder(absolutedEncoder_ID);
    cancoderConfig = new CANcoderConfiguration();

    turningPidController = new PIDController(pid_Kp, pid_Ki, pid_Kd);
    turningPidController.enableContinuousInput(ModuleConstants.pidRangeMin, ModuleConstants.pidRangeMax);

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
    return driveMotor.getVelocity().getValueAsDouble()*ModuleConstants.driveEncoderRPM2MeterPerSec;
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble()*ModuleConstants.driveEncoderRot2Meter;
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
    SwerveModuleState optimizeState = SwerveModuleState.optimize(state, getState().angle);
    turningPidOutput = turningPidController.calculate(getState().angle.getDegrees(), optimizeState.angle.getDegrees());
    turningMotor.set(turningPidOutput);
    driveMotor.set(optimizeState.speedMetersPerSecond);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
