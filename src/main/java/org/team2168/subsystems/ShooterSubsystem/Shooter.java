// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems.ShooterSubsystem;

import java.util.TreeMap;

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
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.MechanismState;
import com.ctre.phoenix6.signals.DifferentialSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase {

  public enum SHOOTING_RPM {
    UP_AGAINST_SPEAKER(1000), // placeholder
    WHITE_LINE(5000),
    RED_LINE(4000),
    UP_AGAINST_AMP(200); // no provided f310 bindings for this on the button bindings paper.

    public double shooterRPS;

    SHOOTING_RPM(double shooterRPS) {
      this.shooterRPS = shooterRPS;
    }

  }

  private static Shooter instance = null;
  private TalonFX leftShooterMotor;
  private TalonFX rightShooterMotor;
  private TalonFXConfiguration firstMotorConfiguration;
  private TalonFXConfiguration secondMotorConfig;
  private Slot0Configs firstMotorGains;
  private CurrentLimitsConfigs currentLimitConfigs;
  private MotorOutputConfigs secondMotorOutputConfigs;

  private FeedbackConfigs firstFeedbackConfigs;

  private MotorOutputConfigs firstOutputConfigs;

  private final double PEAK_FORWARD_DUTY_CYCLE = 1.0; // placeholder
  private final double PEAK_REVERSE_DUTY_CYCLE = -1.0; // placeholder
  private final InvertedValue leftInvert = InvertedValue.Clockwise_Positive;
  private final InvertedValue rightInvert = InvertedValue.CounterClockwise_Positive;

  private double first_kP = 1.00; // placeholder
  private double first_kI = 00; // placeholder
  private double first_kD = 0.0; // placeholder
  private double first_kVolts = 0.05; // placeholder

  private final double GEAR_RATIO = 2.345 / 4.69;
  private final double ACCELERATION = 5 / 60; // placeholder
  private VelocityVoltage velocityVoltage;
  private DutyCycleOut percentOutput;

  private InterpolatingDoubleTreeMap velocityLookup = new InterpolatingDoubleTreeMap() { // calculate motorspeed from //
                                                                                         // distance using a //
                                                                                         // interpolating lookup table.
    {
      put(1.0, 1000.0); // these motorspeeds to meters values are all placeholders, need to actually //
                        // calculate appropriate motorspeed from corresponding distance;
      put(2.0, 2000.0);
      put(3.0, 3000.0);
      put(4.0, 4000.0);
      put(5.0, 5000.0);
      put(6.0, 6000.0);
      put(7.0, 7000.0);
    }
  };

  public Shooter() {
    leftShooterMotor = new TalonFX(Constants.CANDevices.LEFT_SHOOTER_ID);
    rightShooterMotor = new TalonFX(Constants.CANDevices.RIGHT_SHOOTER_ID);
    firstMotorConfiguration = new TalonFXConfiguration();
    secondMotorConfig = new TalonFXConfiguration();
    currentLimitConfigs = new CurrentLimitsConfigs();
    firstOutputConfigs = new MotorOutputConfigs();
    firstMotorGains = new Slot0Configs();
    velocityVoltage = new VelocityVoltage(0.0);
    percentOutput = new DutyCycleOut(0.0);
    secondMotorOutputConfigs = new MotorOutputConfigs();

    leftShooterMotor.clearStickyFaults();
    rightShooterMotor.clearStickyFaults();

    currentLimitConfigs.withSupplyCurrentLimit(20.0);
    currentLimitConfigs.withSupplyCurrentLimitEnable(true);
    currentLimitConfigs.withSupplyCurrentThreshold(25.0);
    currentLimitConfigs.withSupplyTimeThreshold(0.025);

    firstFeedbackConfigs = firstMotorConfiguration.Feedback;

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

    secondMotorOutputConfigs.withDutyCycleNeutralDeadband(0.002);
    secondMotorOutputConfigs.withInverted(rightInvert);
    secondMotorOutputConfigs.withNeutralMode(NeutralModeValue.Brake);
    secondMotorOutputConfigs.withPeakForwardDutyCycle(PEAK_FORWARD_DUTY_CYCLE);
    secondMotorOutputConfigs.withPeakReverseDutyCycle(PEAK_REVERSE_DUTY_CYCLE);

    secondMotorConfig.withSlot0(firstMotorGains);
    secondMotorConfig.withCurrentLimits(currentLimitConfigs);
    secondMotorConfig.withFeedback(firstFeedbackConfigs);
    secondMotorConfig.withMotorOutput(secondMotorOutputConfigs);

    leftShooterMotor.getConfigurator().apply(firstMotorConfiguration);
    rightShooterMotor.getConfigurator().apply(secondMotorConfig);

    velocityVoltage.withAcceleration(ACCELERATION);
    velocityVoltage.withSlot(0);
  }

  /**
   * automaticallly set shooter speed using distance from an object with an
   * interpolation table.
   * 
   * @param distanceFromObject calculated distance from object in meters,
   *                           preferably using a
   *                           camera such as limelight aiming at a apriltag
   */
  public void setMotorSpeedFromDistance(double distanceFromObject) {
    double motorSpeedFromDistance = velocityLookup.get(distanceFromObject);
    setVelocity(motorSpeedFromDistance);
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  /**
   * convert rotations per second to rotations per minute
   * 
   * @param rps rotations per second
   * @return rotations per minute
   */
  public double rpsToRpm(double rps) {
    return rps * 60;
  }

  /**
   * set velocity of shooter using velocityVoltage API
   * 
   * @param velocity as rotations per second
   */
  public void setVelocity(double velocity) {
    leftShooterMotor.setControl(velocityVoltage.withVelocity(velocity * GEAR_RATIO));
    rightShooterMotor.setControl(velocityVoltage.withVelocity((velocity * GEAR_RATIO)-5));
  }

  /**
   * set input of shooter using basic duty cycle out.
   * 
   * @param input power input between -1.0 and 1.0.
   */

  public void setPercentOutput(double input) {
    leftShooterMotor.setControl(percentOutput.withOutput(input));
    rightShooterMotor.setControl(percentOutput.withOutput(input-0.03));
  }

  /**
   * 
   * @return velocity of shooter in rotations per second.
   */
  @Log(name = "shooter velocity in rotations per second", rowIndex = 0, columnIndex = 0)
  public double getVelocity() {
    return (leftShooterMotor.getVelocity().getValue()) / GEAR_RATIO;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
  }
}
