// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.CANDevices;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */  
 private static TalonFX indexerMotor = new TalonFX(CANDevices.INDEXER_MOTOR);
 private static DigitalInput indexerDetector = new DigitalInput(0);

 public static SupplyCurrentLimitConfiguration indexerCurrentLimit;
 public static final boolean ENABLE_CURRENT_LIMIT = true;
 public static final double CONTINUES_CURRENT_LIMIT = 20;
 public static final double TRIGGER_THRESHOLD_LIMIT = 30;
 public static final double TRIGGER_THRESHOLD_TIME = 0.02;
 public static final InvertedValue indexerInvert = InvertedValue.CounterClockwise_Positive;
 private static NeutralModeValue coast = NeutralModeValue.Coast;
 private final DutyCycleOut outputRequest = new DutyCycleOut(0.0);

 private static Indexer instance = null;

  private Indexer() {
    indexerMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());

    var currentConfigurations = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();

    motorConfigs.withInverted(indexerInvert);

    currentConfigurations
    .withSupplyCurrentLimitEnable(ENABLE_CURRENT_LIMIT)
    .withSupplyCurrentLimit(CONTINUES_CURRENT_LIMIT)
    .withSupplyCurrentThreshold(TRIGGER_THRESHOLD_LIMIT)
    .withSupplyTimeThreshold(TRIGGER_THRESHOLD_TIME);

    var indexerConfigurator = indexerMotor.getConfigurator();

    indexerConfigurator.apply(currentConfigurations);
    indexerConfigurator.apply(motorConfigs);
    
    indexerMotor.setNeutralMode(coast);
  }

  public static Indexer getInstance() {
    if(instance == null)
      instance = new Indexer();
    return instance;
  }

    /**
   * sets the speed of the indexer
   * @param speed value should be between 1.0 and -1.0
   */
  public void setDriveIndexer(double speed) {
    indexerMotor.setControl(outputRequest.withOutput(speed));
  }

  /**
   * detects if a note is in the indexer
   * @return if or if not a note is in the indexer
   */
  @Log(name = "Is note present?")
  public boolean isNotePresent() {
   return !indexerDetector.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
