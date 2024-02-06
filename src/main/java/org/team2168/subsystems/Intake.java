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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;


public class Intake extends SubsystemBase {
  private static CANSparkMax intakeRollerOne = new CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless); 
  private static CANSparkMax intakeRollerTwo = new CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless);

  private static Intake instance = null;
  
  private static RelativeEncoder intakeRollerEncoder = intakeRollerOne.getEncoder();

  public Intake getInstance() {
    if(instance == null)
    instance = new Intake();
    return instance;
  }
  
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


  private Intake() {

    intakeRollerOne.restoreFactoryDefaults();
    intakeRollerTwo.restoreFactoryDefaults();

    intakeRollerTwo.follow(intakeRollerOne);
    
  }

  /**
   * sets the speed in percentage
   * @param speed value is between -1.0 and 1.0
   */
  public void setRollerSpeed(double speed) {
    intakeRollerOne.set(speed);
  }
  
  private double RPMToTicksPerOneHundredMS(double speedRPM) {
    return (speedRPM/minuteInHundredMs) * (TICKS_PER_REV/GEAR_RATIO);
  }

  private double TicksPerOneHundredMSToRPM(double ticksPerHundredMs) {
    return ticksPerHundredMs * (GEAR_RATIO/TICKS_PER_REV) * minuteInHundredMs;
  }

@Log(name = "speed (rotations per minutes)", rowIndex = 3, columnIndex = 1)

  public double getSpeedRPM () {
    return TicksPerOneHundredMSToRPM(intakeRollerEncoder.getVelocity());

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
