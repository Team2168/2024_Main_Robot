// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems.ShooterSubsystem;

import org.team2168.Constants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.MechanismState;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private static Shooter instance = null;
  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
  private TalonFXConfiguration firstMotorConfiguration;
  private Slot0Configs firstMotorGains;
  private CurrentLimitsConfigs currentLimitConfigs;

  private FeedbackConfigs firstFeedbackConfigs;

  private MotorOutputConfigs firstOutputConfigs;

  private final double PEAK_FORWARD_DUTY_CYCLE = 0.9; // placeholder
  private final double PEAK_REVERSE_DUTY_CYCLE = -0.9; // placeholder
  private final InvertedValue leftInvert = InvertedValue.Clockwise_Positive;

  private double first_kP = 0.1; // placeholder
  private double first_kI = 0.45; // placeholder
  private double first_kD = 0.001; // placeholder
  private double first_kVolts = 0.12; // placeholder

  private final double GEAR_RATIO = 2.345;
  private final double ACCELERATION = rpmToRps(25); // placeholder
  private VelocityVoltage velocityVoltage;
  private DutyCycleOut percentOutput;
  private FlywheelSim flywheelSim = new FlywheelSim(DCMotor.getFalcon500(2), GEAR_RATIO, 1.161E-7);

  private InterpolatingTreeMap velocityLookupTable; // judge velocity of shooter based on distance with interpolating
                                                    // look-up table

  public Shooter() {
    leftShooterMotor = new TalonFX(Constants.SHOOTER_MOTOR_CONSTANTS.LEFT_SHOOTER_ID);
    rightShooterMotor = new TalonFX(Constants.SHOOTER_MOTOR_CONSTANTS.RIGHT_SHOOTER_ID);
    firstMotorConfiguration = new TalonFXConfiguration();
    currentLimitConfigs = new CurrentLimitsConfigs();
    firstFeedbackConfigs = new FeedbackConfigs();
    firstOutputConfigs = new MotorOutputConfigs();
    firstMotorGains = new Slot0Configs();
    velocityVoltage = new VelocityVoltage(0.0);
    percentOutput = new DutyCycleOut(0.0);

    leftShooterMotor.clearStickyFaults();
    rightShooterMotor.clearStickyFaults();

    currentLimitConfigs.withSupplyCurrentLimit(20.0);
    currentLimitConfigs.withSupplyCurrentLimitEnable(true);
    currentLimitConfigs.withSupplyCurrentThreshold(25.0);
    currentLimitConfigs.withSupplyTimeThreshold(0.025);

    firstFeedbackConfigs = firstMotorConfiguration.Feedback;
    firstFeedbackConfigs.withSensorToMechanismRatio(12.8);

    firstOutputConfigs.withDutyCycleNeutralDeadband(0.002);
    firstOutputConfigs.withInverted(leftInvert);
    firstOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
    firstOutputConfigs.withPeakForwardDutyCycle(PEAK_FORWARD_DUTY_CYCLE);
    firstOutputConfigs.withPeakReverseDutyCycle(PEAK_REVERSE_DUTY_CYCLE);

    firstMotorGains.withKP(first_kP);
    firstMotorGains.withKI(first_kI);
    firstMotorGains.withKD(first_kD);
    firstMotorGains.withKV(first_kVolts);

    firstMotorConfiguration.withSlot0(firstMotorGains);
    firstMotorConfiguration.withCurrentLimits(currentLimitConfigs);
    firstMotorConfiguration.withFeedback(firstFeedbackConfigs);
    firstMotorConfiguration.withMotorOutput(firstOutputConfigs);

    leftShooterMotor.getConfigurator().apply(firstMotorConfiguration);

    rightShooterMotor.setControl(new Follower(leftShooterMotor.getDeviceID(), true));
    velocityVoltage.withAcceleration(rpmToRps(ACCELERATION));
    velocityVoltage.withSlot(0);
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  public double rpmToRps(double rpm) {
    return (rpm * GEAR_RATIO) / 60;
  }

  public void setVelocity(double velocity) {
    leftShooterMotor.setControl(velocityVoltage.withVelocity(rpmToRps(velocity)));
    
  }

  public void setPercentOutput(double input) {
    leftShooterMotor.setControl(percentOutput.withOutput(input));
  }

  public StatusSignal<Double> getVelocity() {
    return leftShooterMotor.getVelocity();
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    flywheelSim.setInputVoltage(1);
  }
}
