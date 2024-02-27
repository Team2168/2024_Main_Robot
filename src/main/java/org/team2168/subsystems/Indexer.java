// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import java.lang.System.Logger;

import org.team2168.utils.TalonFXHelper;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private static DigitalInput detector;
private static TalonFXHelper motor;

 private static TalonFX IndexerMotor = new TalonFX(24);

 public static SupplyCurrentLimitConfiguration indexerCurrentLimit;
  public static final boolean ENABLE_CURRENT_LIMIT = true;
 public static final double CONTINUES_CURRENT_LIMIT = 20;
 public static final double TRIGGER_THRESHOLD_LIMIT = 30;
 public static final double TRIGGER_THRESHOLD_TIME = 0.02;
 public static final InvertedValue indexerInvert = InvertedValue.Clockwise_Positive;
  
private static Indexer instance = null;

  private Indexer() {
    detector = new DigitalInput(14); //placeholders for the time bieng
    motor = new TalonFXHelper(24);
    var currentConfigurations = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();
IndexerMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
motorConfigs.withInverted(indexerInvert);
var indexerConfigurator = IndexerMotor.getConfigurator();

currentConfigurations
.withSupplyCurrentLimitEnable(ENABLE_CURRENT_LIMIT)
.withSupplyCurrentLimit(CONTINUES_CURRENT_LIMIT)
.withSupplyCurrentThreshold(TRIGGER_THRESHOLD_LIMIT)
.withSupplyTimeThreshold(TRIGGER_THRESHOLD_TIME);

indexerConfigurator.apply(currentConfigurations);
motor.setNeutralMode(NeutralMode.Brake);

motor.configOpenLoopStatusFrameRates();
  }

  private static Indexer getInstance() {
    if(instance == null)
      instance = new Indexer();
    return instance;
  }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
