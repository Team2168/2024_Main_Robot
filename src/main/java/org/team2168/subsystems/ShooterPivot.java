// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {
  
  TalonFX pivotMotor;
  MotionMagicVoltage motionMagic;
  MotionMagicConfigs motionMagicConfigs;
  TalonFXConfiguration pivotMotorConfigs;
  Slot0Configs pivotMotorGains;
  
  public ShooterPivot() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
