// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems.ShooterSubsystem;

import org.team2168.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivot extends SubsystemBase {

  private TalonFX pivotMotor;
  private MotionMagicVoltage motionMagic;
  private MotionMagicConfigs motionMagicConfigs;
  private TalonFXConfiguration pivotMotorConfigs;
  private Slot0Configs pivotMotorGains;
  private FeedbackConfigs feedbackConfig;
  private MotorOutputConfigs motorOutputConfig;
  private CurrentLimitsConfigs motorCurrentConfig;
  private final double GEAR_RATIO = 0.0; // placeholder
  private final double MINIMUM_LIMIT_ANGLE = Units.degreesToRotations(0);// placeholder for softlimit
  private final double MAXIMUM_LIMIT_ANGLE = Units.degreesToRotations(90); // placeholder for softlimit
  private final double STOW_ANGLE = Units.degreesToRotations(80);
  private final double PEAK_FORWARD_OUTPUT = 20;
  private final double PEAK_REVERSE_OUTPUT = 20;
  private final InvertedValue pivotInvert = InvertedValue.Clockwise_Positive;
  private double supplyCurrentLimit = 20; // placeholder
  private boolean supplyCurrentLimitEnable = true; // placeholder
  private double supplyCurrentThreshold = 20.05;
  private double supplyTimeThreshold = 0.02;
  private double kP = 1.00; //placeholder
  private double kI = 0.04; //placeholder
  private double kD = 0.0025; //placeholder

  public ShooterPivot() {
    pivotMotor = new TalonFX(Constants.SHOOTER_MOTOR_CONSTANTS.SHOOTER_PIVOT_ID);
    pivotMotorConfigs = new TalonFXConfiguration();
    motionMagic = new MotionMagicVoltage(0.0);
    motionMagic.withSlot(0);
    motionMagicConfigs = pivotMotorConfigs.MotionMagic;
    pivotMotorGains = pivotMotorConfigs.Slot0;
    feedbackConfig = pivotMotorConfigs.Feedback;
    motorOutputConfig = pivotMotorConfigs.MotorOutput;
    motorCurrentConfig = pivotMotorConfigs.CurrentLimits;

    motionMagicConfigs.withMotionMagicAcceleration(degreesPerSecondToRotationsPerSecond(36)); //placeholder
    motionMagicConfigs.withMotionMagicCruiseVelocity(degreesPerSecondToRotationsPerSecond(18)); //placeholder
    motionMagicConfigs.withMotionMagicJerk(degreesPerSecondToRotationsPerSecond(0.03)); //placeholder

    motorOutputConfig.withInverted(pivotInvert);
    motorOutputConfig.withNeutralMode(NeutralModeValue.Brake);

    motorCurrentConfig.withSupplyCurrentLimit(supplyCurrentLimit);
    motorCurrentConfig.withSupplyCurrentLimitEnable(supplyCurrentLimitEnable);
    motorCurrentConfig.withSupplyCurrentThreshold(supplyCurrentThreshold);
    motorCurrentConfig.withSupplyTimeThreshold(supplyCurrentThreshold);

    pivotMotorGains.withKP(kP);
    pivotMotorGains.withKI(kI);
    pivotMotorGains.withKD(kD);
    pivotMotorGains.withGravityType(GravityTypeValue.Arm_Cosine); //shooterhood is basically an arm.
    
    feedbackConfig.withSensorToMechanismRatio(12.8);

    pivotMotor.getConfigurator().apply(pivotMotorConfigs);

  }

  public double rpmToRpMM(double rpm) {
    return (rpm * GEAR_RATIO) / 60;
  }

  public double degreesPerSecondToRotationsPerSecond(double degreesPerSecond) {
    return degreesPerSecond / 360;
  }

  public void setPositionDegrees(double degrees) {
    pivotMotor.setControl(motionMagic.withPosition(Units.degreesToRotations(degrees)));
  }

  public void setToStowAngle() {
    pivotMotor.setControl(motionMagic.withPosition(Units.degreesToRotations(STOW_ANGLE)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
