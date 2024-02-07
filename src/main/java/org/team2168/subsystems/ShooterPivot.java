// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {

  private TalonFX pivotMotor;
  private MotionMagicVoltage motionMagic;
  private MotionMagicConfigs motionMagicConfigs;
  private TalonFXConfiguration pivotMotorConfigs;
  private Slot0Configs pivotMotorGains;
  private FeedbackConfigs feedbackConfig;
  private MotorOutputConfigs motorOutputConfig;
  private final double GEAR_RATIO = 0.0; // placeholder
  private final double MINIMUM_LIMIT_ANGLE = degreesToTicks(0.0); //placeholder for softlimit
  private final double MAXIMUM_LIMIT_ANGLE = degreesToTicks(75); //placeholder for softlimit

  public ShooterPivot() {
    pivotMotor = new TalonFX(Constants.SHOOTER_MOTOR_CONSTANTS.SHOOTER_PIVOT_ID);
    pivotMotorConfigs = new TalonFXConfiguration();
    motionMagic = new MotionMagicVoltage(0.0);
    motionMagic.withPosition(0.0);
    motionMagicConfigs = pivotMotorConfigs.MotionMagic;
    pivotMotorGains = pivotMotorConfigs.Slot0;
    feedbackConfig = pivotMotorConfigs.Feedback;
    motorOutputConfig = pivotMotorConfigs.MotorOutput;

    feedbackConfig.withSensorToMechanismRatio(12.8);

    pivotMotor.getConfigurator().apply(pivotMotorConfigs);

  }

  public double degreesToTicks(double degrees) {
    return (degrees / 360) * GEAR_RATIO * 2048;
  }

  public void setPositionDegrees(double degrees) {
    pivotMotor.setControl(motionMagic.withPosition(degreesToTicks(degrees)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motionMagic.withSlot(0);
  }
}
