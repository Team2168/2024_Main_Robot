// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {

  private static TalonFX intakePivotOne = new TalonFX(0); // leader motor
  private static TalonFX intakePivotTwo = new TalonFX(0); // follower motor
  private static IntakePivot instance = null;

  public IntakePivot getInstance() {
    if(instance == null)
    instance = new IntakePivot();
    return instance;
  }

  private static InvertedValue intakeInvertOne = InvertedValue.Clockwise_Positive; //TODO: check value
  private static InvertedValue intakeInvertTwo = InvertedValue.CounterClockwise_Positive;

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

  private IntakePivot() {
    intakePivotOne.getConfigurator().apply(new TalonFXConfiguration()); //resets leader motor to its factory default
    intakePivotTwo.getConfigurator().apply(new TalonFXConfiguration()); //resets follower motor to its factory default

    var currentConfigs = new CurrentLimitsConfigs();
    var PIDconfigs = new Slot0Configs();
    var motionMagicConfigs = new MotionMagicConfigs();
    var leaderMotorConfigs = new MotorOutputConfigs();
    var followerMotorConfigs = new MotorOutputConfigs();

    leaderMotorConfigs.withInverted(intakeInvertOne);
    leaderMotorConfigs.withDutyCycleNeutralDeadband(neutralDeadband);
   // followerMotorConfigs.withInverted(intakeInvertTwo);

    currentConfigs
      .withSupplyCurrentLimitEnable(ENABLE_CURRENT_LIMIT)
      .withSupplyCurrentLimit(CONTINUOUS_CURRENT_LIMIT)
      .withSupplyCurrentThreshold(TRIGGER_THRESHOLD_LIMIT)
      .withSupplyTimeThreshold(TRIGGER_THRESHOLD_TIME);
    //intakePivotConfig.withCurrentLimits(currentLimitConfig);
    //intakePivotConfig.supplyCurrLimit = talonCurrentLimit;

    PIDconfigs
      .withKP(kP)
      .withKI(kI)
      .withKD(kD);

    motionMagicConfigs
      .withMotionMagicAcceleration(motionMagicAcceleration)
      .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
      .withMotionMagicExpo_kA(kA)
      .withMotionMagicExpo_kV(kV);

    var intakeRaiseAndLowerOne = intakePivotOne.getConfigurator();
    var intakeRaiseAndLowerTwo = intakePivotTwo.getConfigurator();
    
    intakeRaiseAndLowerOne.apply(leaderMotorConfigs);
    intakeRaiseAndLowerOne.apply(currentConfigs);
    intakeRaiseAndLowerOne.apply(PIDconfigs);

    intakeRaiseAndLowerTwo.apply(followerMotorConfigs);
    intakeRaiseAndLowerTwo.apply(currentConfigs);
    
    intakePivotTwo.setControl(new Follower(intakePivotOne.getDeviceID(), true));
    // intakeRaiseAndLower.apply(motionMagicConfigs);
    
    
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
    intakePivotOne.setControl(motionMagicVoltage.withPosition((degreesToRot(demand))));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
