// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static CANSparkMax climberMotor = new CANSparkMax(Constants.CANDevices.CLIMBER_MOTOR, CANSparkLowLevel.MotorType.kBrushed);
  private static RelativeEncoder climberEncoder = climberMotor.getAlternateEncoder(50);
  private static Climber instance = null;

  private int currentLimit = 30;
  private IdleMode idleMode = IdleMode.kBrake;
  private boolean isInverted = false;

  /** Creates a new Climber. */
  public Climber() {
    climberMotor.setSmartCurrentLimit(currentLimit);
    climberMotor.setIdleMode(idleMode);
    climberMotor.setInverted(isInverted);

    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 825);
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
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
