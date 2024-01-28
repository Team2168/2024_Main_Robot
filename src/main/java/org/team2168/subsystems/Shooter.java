// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter instance = null;
  private TalonFX firstShooterMotor;
  private TalonFX secondShooterMotor;
  private TalonFXConfigurator firstShooterConfig;
  private TalonFXConfigurator secondShooterConfig;
  private Slot0Configs shooterGains;
  private ClosedLoopGeneralConfigs closedLoopConfigs;
  private CurrentLimitsConfigs currentLimitConfigs;
  private DifferentialSensorsConfigs sensorConfigs;
  private FeedbackConfigs feedbackConfigs;
  private MotorOutputConfigs motorOutputConfigs;

  public Shooter() {
    firstShooterMotor = new TalonFX(Constants.MOTOR_CONSTANTS.FIRST_SHOOTER_ID);
    secondShooterMotor = new TalonFX(Constants.MOTOR_CONSTANTS.SECOND_SHOOTER_ID);
    firstShooterConfig = new TalonFXConfigurator(Constants.MOTOR_CONSTANTS.FIRST_SHOOTER_CONFIG_ID);
    secondShooterConfig = new TalonFXConfigurator(Constants.MOTOR_CONSTANTS.FIRST_SHOOTER_CONFIG_ID);
    closedLoopConfigs = new ClosedLoopGeneralConfigs();
    currentLimitConfigs = new CurrentLimitsConfigs();
    sensorConfigs = new DifferentialSensorsConfigs();
    feedbackConfigs = new FeedbackConfigs();
    motorOutputConfigs = new MotorOutputConfigs();
    shooterGains = new Slot0Configs();

    firstShooterMotor.clearStickyFaults();
    secondShooterMotor.clearStickyFaults();

    firstShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    secondShooterMotor.setNeutralMode(NeutralModeValue.Brake);

    firstShooterMotor.setControl(new Follower(Constants.MOTOR_CONSTANTS.SECOND_SHOOTER_ID, true));

  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
