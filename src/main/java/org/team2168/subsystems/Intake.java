// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;


public class Intake extends SubsystemBase {
  private static TalonFX intakeRoller = new TalonFX(0);
  private static Intake instance = null;
  private static TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();

  public Intake getInstance() {
    if(instance == null)
    instance = new Intake();
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
  private static CurrentLimitsConfigs currentLimitConfig;
  private double neutralDeadband = 0.05;
  private double maxForwardOutput = 1;
  private double maxBackwardOutput = -1;

  private final double TICKS_PER_REV = 2048;
  private final double GEAR_RATIO = 0; // TODO: Add later


  /*
   * In intake constructor configures motor to have certain settings 
   * Using the TalonFXConfig variable, gives it different values
   * After putting all parameters into configuration variable, pushes it all into the intake motor
   */
  private Intake() {
    intakeRoller.getConfigurator().apply( new TalonFXConfiguration()); //resets motor to its factory default
    var motorConfigs = new MotorOutputConfigs();
    var currentConfigs = new CurrentLimitsConfigs();

    motorConfigs.withInverted(intakeInvert);
    motorConfigs.withDutyCycleNeutralDeadband(neutralDeadband);

    currentConfigs
      .withSupplyCurrentLimitEnable(ENABLE_CURRENT_LIMIT)
      .withSupplyCurrentLimit(CONTINUOUS_CURRENT_LIMIT)
      .withSupplyCurrentThreshold(TRIGGER_THRESHOLD_LIMIT)
      .withSupplyTimeThreshold(TRIGGER_THRESHOLD_TIME);
    //intakeRollerConfig.withCurrentLimits(currentLimitConfig);
    //intakeRollerOneConfig.supplyCurrLimit = talonCurrentLimit;

    var intakeRolle = intakeRoller.getConfigurator();
    intakeRolle.apply(motorConfigs);
    intakeRolle.apply(currentConfigs);
    
  }

  /**
   * sets the speed in percentage
   * @param speed value is between -1.0 and 1.0
   */
  private static void setRollerSpeed(double speed) {
    intakeRoller.set(speed);
  }
  
  private double RPMToTicksPerOneHundredMS(double speedRPM) {
    return (speedRPM/minuteInHundredMs) * (TICKS_PER_REV/GEAR_RATIO);
  }

  private double TicksPerOneHundredMSToRPM(double ticksPerHundredMs) {
    return ticksPerHundredMs * (GEAR_RATIO/TICKS_PER_REV) * minuteInHundredMs;
  }

@Log(name = "speed (rotations per minutes)", rowIndex = 3, columnIndex = 1);

  public double getSpeedRPM () {
    return TicksPerOneHundredMSToRPM(intakeRoller.getVelocity().getValueAsDouble());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
