// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static CANSparkMax climberMotor = new CANSparkMax(Constants.CANDevices.CLIMBER_MOTOR, CANSparkLowLevel.MotorType.kBrushed);
  private static Climber instance = null;

  private int currentLimit = 20;
  private IdleMode idleMode = IdleMode.kBrake;
  private boolean isInverted = false;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor.setSmartCurrentLimit(currentLimit);
    climberMotor.setIdleMode(idleMode);
    climberMotor.setInverted(isInverted);
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  public void setSpeed(double speed) {
    double var = MathUtil.clamp(speed, -1.0, 1.0);
    climberMotor.set(var);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
