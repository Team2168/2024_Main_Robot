// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems.ShooterSubsystem;

import org.team2168.Constants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterPivot extends SubsystemBase {
  public enum SHOOTING_ANGLE {
    UP_AGAINST_SPEAKER(87.5), // placeholder
    WHITE_LINE(25.0),
    RED_LINE(35.0),
    UP_AGAINST_AMP(89.0); //no provided f310 bindings for this on the button bindings paper.

    public double shooterAngle;

    SHOOTING_ANGLE(double shooterAngle) {
      this.shooterAngle = shooterAngle;
    }

  }

  private static ShooterPivot shooterPivot = null;
  private TalonFX pivotMotor;
  private MotionMagicVoltage motionMagic;
  private MotionMagicConfigs motionMagicConfigs;
  private TalonFXConfiguration pivotMotorConfigs;
  private Slot0Configs pivotMotorGains;
  private FeedbackConfigs feedbackConfig;
  private MotorOutputConfigs motorOutputConfig;
  private CurrentLimitsConfigs motorCurrentConfig;
  private final double GEAR_RATIO = 45.024; // placeholder
  private final double MINIMUM_LIMIT_ANGLE = Units.degreesToRotations(0);// placeholder for softlimit
  private final double MAXIMUM_LIMIT_ANGLE = Units.degreesToRotations(90); // placeholder for softlimit
  private final double STOW_ANGLE = Units.degreesToRotations(80);
  private final double PEAK_FORWARD_OUTPUT = 1.0;
  private final double PEAK_REVERSE_OUTPUT = -1.0;
  private final InvertedValue pivotInvert = InvertedValue.Clockwise_Positive;
  private double supplyCurrentLimit = 20; // placeholder
  private boolean supplyCurrentLimitEnable = true; // placeholder
  private double supplyCurrentThreshold = 20.05;
  private double supplyTimeThreshold = 0.02;
  private double kP = 1.00; // placeholder
  private double kI = 0.04; // placeholder
  private double kD = 0.0025; // placeholder
  private double kG = 0.0025; // placeholder.
  private SoftwareLimitSwitchConfigs rotationLimits;
  private DutyCycleOut percentOutput;

  private InterpolatingDoubleTreeMap shooterAngleLookup = new InterpolatingDoubleTreeMap() { // calculate motorspeed                                                                                            // from  // distance using a // interpolating lookup table.
    {
      put(1.0, 80.0); // these motorspeeds to meters values are all placeholders, need to actually calculate appropriate motorspeed from corresponding distance;
      put(2.0, 70.0);
      put(3.0, 60.0);
      put(4.0, 50.0);
      put(5.0, 40.0);
      put(6.0, 30.0);
      put(7.0, 20.0);
    }
  };

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
    rotationLimits = pivotMotorConfigs.SoftwareLimitSwitch;
    percentOutput = new DutyCycleOut(0.0);

    motionMagicConfigs.withMotionMagicAcceleration(degreesPerSecondToRotationsPerSecond(36)); // placeholder
    motionMagicConfigs.withMotionMagicCruiseVelocity(degreesPerSecondToRotationsPerSecond(18)); // placeholder
    motionMagicConfigs.withMotionMagicJerk(Units.degreesToRotations(0.0025)); //modifying jerk appears to be a necessary config for motion magic according to MotionMagicVoltage.
    // motionMagicConfigs.withMotionMagicJerk(degreesPerSecondToRotationsPerSecond(0.03));
    // //placeholder

    motorOutputConfig.withInverted(pivotInvert);
    motorOutputConfig.withNeutralMode(NeutralModeValue.Brake);
    motorOutputConfig.withPeakForwardDutyCycle(PEAK_FORWARD_OUTPUT);
    motorOutputConfig.withPeakReverseDutyCycle(PEAK_REVERSE_OUTPUT);

    motorCurrentConfig.withSupplyCurrentLimit(supplyCurrentLimit);
    motorCurrentConfig.withSupplyCurrentLimitEnable(supplyCurrentLimitEnable);
    motorCurrentConfig.withSupplyCurrentThreshold(supplyCurrentThreshold);
    motorCurrentConfig.withSupplyTimeThreshold(supplyCurrentThreshold);

    pivotMotorGains.withKP(kP);
    pivotMotorGains.withKI(kI);
    pivotMotorGains.withKD(kD);
    pivotMotorGains.withKG(kG);

    feedbackConfig.withSensorToMechanismRatio(12.8);

    rotationLimits.withForwardSoftLimitEnable(true);
    rotationLimits.withReverseSoftLimitEnable(true);
    rotationLimits.withForwardSoftLimitThreshold(MAXIMUM_LIMIT_ANGLE);
    rotationLimits.withReverseSoftLimitThreshold(MINIMUM_LIMIT_ANGLE);

    pivotMotor.getConfigurator().apply(pivotMotorConfigs);
  }

  public static ShooterPivot getInstance() {
    if (shooterPivot == null) {
      shooterPivot = new ShooterPivot();
    }
    return shooterPivot;
  }

  /**
   * control motor using basic DutyCycle output.
   * 
   * @param input input between -1.0-+1.0
   */
  public void setPercentOutput(double input) {
    pivotMotor.setControl(percentOutput.withOutput(input));
  }

  /**
   * convert degrees per second to rotations per second.
   * 
   * @param degreesPerSecond
   * @return rotations per second.
   */
  public double degreesPerSecondToRotationsPerSecond(double degreesPerSecond) {
    return degreesPerSecond / 360;
  }

  /**
   * Control shooterPivot angle with motionMagic.
   * 
   * @param degrees
   */
  public void setPositionDegrees(double degrees) {
    pivotMotor.setControl(motionMagic.withPosition(Units.degreesToRotations(degrees)));
  }

  /** Preset the stow angle of the shooter hood (80 degrees) */
  public void setToStowAngle() {
    pivotMotor.setControl(motionMagic.withPosition(Units.degreesToRotations(STOW_ANGLE)));
  }
  /**
   * automaticallly set shooter hood angle using distance from an object with an
   * interpolation table.
   * @param distanceFromObject calculated distance from object in meters, preferably using a
   *                           camera such as limelight aiming at a apriltag
   */
  public void setShooterAngleFromDistance(double distanceFromObject) {
    double shooterAngle = shooterAngleLookup.get(distanceFromObject);
    setPositionDegrees(shooterAngle);
  }

  /**
   * 
   * @return angle of shooter.
   */
  @Log(name = "shooter hood angle (degrees)", rowIndex = 0, columnIndex = 1)
  public double getAngle() {
    return Units.rotationsToDegrees(pivotMotor.getPosition().getValue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
