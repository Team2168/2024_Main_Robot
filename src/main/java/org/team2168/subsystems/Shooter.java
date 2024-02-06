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
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.MechanismState;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter instance = null;
  private TalonFX firstShooterMotor;
  private TalonFX secondShooterMotor;
  private TalonFXConfigurator firstShooterConfig;
  private TalonFXConfigurator secondShooterConfig;
  private Slot0Configs firstMotorGains;
  private Slot1Configs secondMotorGains;
  private ClosedLoopGeneralConfigs closedLoopConfigs;
  private CurrentLimitsConfigs currentLimitConfigs;
  private DifferentialSensorsConfigs firstSensorConfigs;
  private DifferentialSensorsConfigs secondSensorConfigs;

  private FeedbackConfigs firstFeedbackConfigs;
  private FeedbackConfigs secondFeedbackConfigs;

  private MotorOutputConfigs firstOutputConfigs;
  private MotorOutputConfigs secondOutputConfigs;

  private final DeviceIdentifier FIRST_SHOOTER_CONFIG_ID = new DeviceIdentifier(); // placeholder
  private final DeviceIdentifier SECOND_SHOOTER_CONFIG_ID = new DeviceIdentifier(); // placeholder
  private final int FIRST_TALON_FX_SENSOR_ID_NUMBER = 1; // PLACEHOLDER
  private final int SECOND_TALON_FX_SENSOR_ID_NUMBER = 2;
  private final double PEAK_FORWARD_DUTY_CYCLE = 10.00;
  private final double PEAK_REVERSE_DUTY_CYCLE = 10.00;
  private final InvertedValue leftInvert = InvertedValue.Clockwise_Positive;
  private final InvertedValue rightInvert = InvertedValue.CounterClockwise_Positive;

  private double first_kP = 0.1; // placeholder
  private double first_kI = 0.45; // placeholder
  private double first_kD = 0.001; // placeholder
  private double first_kVolts = 0.12; // placeholder

  private double second_kP = 0.1; // placeholder
  private double second_kI = 0.45; // placeholder
  private double second_kD = 0.001; // placeholder
  private double second_kVolts = 0.12; // placeholder

  private final double GEAR_RATIO = 0;
  private VelocityVoltage velocityVoltage;

  public Shooter() {
    firstShooterMotor = new TalonFX(Constants.SHOOTER_MOTOR_CONSTANTS.FIRST_SHOOTER_ID);
    secondShooterMotor = new TalonFX(Constants.SHOOTER_MOTOR_CONSTANTS.SECOND_SHOOTER_ID);
    firstShooterConfig = new TalonFXConfigurator(FIRST_SHOOTER_CONFIG_ID);
    secondShooterConfig = new TalonFXConfigurator(FIRST_SHOOTER_CONFIG_ID);
    closedLoopConfigs = new ClosedLoopGeneralConfigs();
    currentLimitConfigs = new CurrentLimitsConfigs();
    firstSensorConfigs = new DifferentialSensorsConfigs();
    secondSensorConfigs = new DifferentialSensorsConfigs();
    firstFeedbackConfigs = new FeedbackConfigs();
    secondFeedbackConfigs = new FeedbackConfigs();
    firstOutputConfigs = new MotorOutputConfigs();
    secondOutputConfigs = new MotorOutputConfigs();
    firstMotorGains = new Slot0Configs();
    secondMotorGains = new Slot1Configs();
    velocityVoltage = new VelocityVoltage(0.0);

    firstShooterMotor.clearStickyFaults();
    secondShooterMotor.clearStickyFaults();

    currentLimitConfigs.withSupplyCurrentLimit(20.0);
    currentLimitConfigs.withSupplyCurrentLimitEnable(true);
    currentLimitConfigs.withSupplyCurrentThreshold(25.0);
    currentLimitConfigs.withSupplyTimeThreshold(0.025);

    firstSensorConfigs.withDifferentialTalonFXSensorID(FIRST_TALON_FX_SENSOR_ID_NUMBER);
    firstFeedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    secondSensorConfigs.withDifferentialTalonFXSensorID(SECOND_TALON_FX_SENSOR_ID_NUMBER);
    firstFeedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

    firstOutputConfigs.withDutyCycleNeutralDeadband(0.002);
    firstOutputConfigs.withInverted(leftInvert);
    firstOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
    firstOutputConfigs.withPeakForwardDutyCycle(PEAK_FORWARD_DUTY_CYCLE);
    firstOutputConfigs.withPeakReverseDutyCycle(PEAK_REVERSE_DUTY_CYCLE);

    secondOutputConfigs.withDutyCycleNeutralDeadband(0.002);
    secondOutputConfigs.withInverted(rightInvert);
    secondOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
    secondOutputConfigs.withPeakForwardDutyCycle(PEAK_FORWARD_DUTY_CYCLE);
    secondOutputConfigs.withPeakReverseDutyCycle(PEAK_REVERSE_DUTY_CYCLE);

    firstMotorGains.withKP(first_kP);
    firstMotorGains.withKI(first_kI);
    firstMotorGains.withKD(first_kD);
    firstMotorGains.withKV(first_kVolts);

    secondMotorGains.withKP(first_kP);
    secondMotorGains.withKI(first_kI);
    secondMotorGains.withKD(first_kD);
    secondMotorGains.withKV(first_kVolts);

    firstShooterConfig.apply(closedLoopConfigs);
    firstShooterConfig.apply(currentLimitConfigs);
    firstShooterConfig.apply(firstSensorConfigs);
    firstShooterConfig.apply(firstFeedbackConfigs);
    firstShooterConfig.apply(firstOutputConfigs);
    firstShooterConfig.apply(firstMotorGains);

    secondShooterConfig.apply(closedLoopConfigs);
    secondShooterConfig.apply(currentLimitConfigs);
    secondShooterConfig.apply(secondSensorConfigs);
    secondShooterConfig.apply(secondFeedbackConfigs);
    secondShooterConfig.apply(secondOutputConfigs);
    secondShooterConfig.apply(secondMotorGains);

    secondShooterMotor.setControl(new Follower(Constants.SHOOTER_MOTOR_CONSTANTS.SECOND_SHOOTER_ID, true));
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  public void setVelocity(double velocity) {
    firstShooterMotor.setControl(velocityVoltage.withVelocity(velocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
