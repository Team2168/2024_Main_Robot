// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeUpAndDown extends SubsystemBase {
  /** Creates a new IntakeUpAndDown. */

  private static TalonFX intakeUpAndDown = new TalonFX(0);
  private static IntakeUpAndDown instance = null;

  public IntakeUpAndDown getInstance() {
    if(instance == null)
    instance = new IntakeUpAndDown();
    return instance;
  }

  private static InvertedValue intakeInvert = InvertedValue.Clockwise_Positive; //TODO: check value
  
  private static int intakeTimeoutMs = 30;
  private static double peakOutput = 1.0;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20.0;
  private final double TRIGGER_THRESHOLD_LIMIT = 25;
  private final double TRIGGER_THRESHOLD_TIME = 0.2;
  private final double minuteInHundredMs = 600.0;
  private double neutralDeadband = 0.05;
  private double maxForwardOutput = 1;
  private double maxBackwardOutput = -1;
  final double MIN_ANGLE = 0;
  final double MAX_ANGLE = 120;
  private double motionMagicAcceleration = 5.0;
  private double motionMagicCruiseVelocity = 10.0;
  private double kV = 0.12;
  private double kA = 0.1;
  final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  private final double TICKS_PER_REV = 2048;
  private final static double GEAR_RATIO = 0;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private IntakeUpAndDown() {
    intakeUpAndDown.getConfigurator().apply( new TalonFXConfiguration()); //resets motor to its factory default
    var motorConfigs = new MotorOutputConfigs();
    var currentConfigs = new CurrentLimitsConfigs();

    var PIDconfigs = new Slot0Configs();
    motorConfigs.withInverted(intakeInvert);
    motorConfigs.withDutyCycleNeutralDeadband(neutralDeadband);

    currentConfigs
      .withSupplyCurrentLimitEnable(ENABLE_CURRENT_LIMIT)
      .withSupplyCurrentLimit(CONTINUOUS_CURRENT_LIMIT)
      .withSupplyCurrentThreshold(TRIGGER_THRESHOLD_LIMIT)
      .withSupplyTimeThreshold(TRIGGER_THRESHOLD_TIME);
    //intakeRollerConfig.withCurrentLimits(currentLimitConfig);
    //intakeRollerOneConfig.supplyCurrLimit = talonCurrentLimit;

    PIDconfigs
      .withKP(kP)
      .withKI(kI)
      .withKD(kD);

    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs
      .withMotionMagicAcceleration(motionMagicAcceleration)
      .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
      .withMotionMagicExpo_kA(kA)
      .withMotionMagicExpo_kV(kV);

    var intakeRaiseAndLower = intakeUpAndDown.getConfigurator();
    intakeRaiseAndLower.apply(motorConfigs);
    intakeRaiseAndLower.apply(currentConfigs);
    intakeRaiseAndLower.apply(PIDconfigs);
    
  }

  /**
   * 
   * @param ticks talonfx motor conversion
   * @return intakeupanddown is relative to its lowered position (0.0)
   */
  public static double rotToDegrees(double rot) {
    return (rot) / GEAR_RATIO * 360.0;
  }

  /**
   * commands the intakeupanddown to a specific position relative to the zero positon (0.0)
   * 
   * @param degrees amount of degrees/angles to move intakeupanddown up
   */
  public static double degreesToRot(double degrees) {
    return (degrees/360) * GEAR_RATIO;
  }

  private void setIntakePosition(double degrees) {
    var demand = MathUtil.clamp(degrees, MIN_ANGLE, MAX_ANGLE);
    intakeUpAndDown.setControl(motionMagicVoltage.withPosition((degreesToRot(demand))));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
