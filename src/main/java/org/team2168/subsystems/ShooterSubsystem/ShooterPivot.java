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
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ShooterPivot extends SubsystemBase implements Loggable {
  public enum SHOOTING_ANGLE {
    UP_AGAINST_SPEAKER(60.0), // placeholder
    SUBWOOFER_SIDE(59.0),
    WHITE_LINE(60.0),
    RED_LINE(41.0),
    UP_AGAINST_AMP(62.0), //no provided f310 bindings for this on the button bindings paper.
    STARTING_ZONE_LINE(46.0),
    STAGE_LINE(40.25),
    TRAP_SHOT(57.3);

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
  private final double GEAR_RATIO = (45.024/4.69) * 2.0; // placeholder
  private final double MINIMUM_LIMIT_ANGLE = degreesToRotation(25.0);// placeholder for softlimit
  private final double MAXIMUM_LIMIT_ANGLE = degreesToRotation(62.0); // placeholder for softlimit
  private final double STOW_ANGLE = degreesToRotation(62.0); //actual value is suppost to be 80 degrees.
  private final double PEAK_FORWARD_OUTPUT = 1.0;
  private final double PEAK_REVERSE_OUTPUT = -1.0;
  private final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;
  private double supplyCurrentLimit = 20; // placeholder
  private boolean supplyCurrentLimitEnable = true; // placeholder
  private double supplyCurrentThreshold = 20.05;
  private double supplyTimeThreshold = 0.02;
  private double kP = 25.0; // placeholder
  private double kI = 30.0; // placeholder
  private double kD = 0.0; // placeholder
  private double kG = 0.0; // placeholder, negative because we need down force to counteract tension.
  private double kS = 0.0;
  private SoftwareLimitSwitchConfigs rotationLimits;
  private DutyCycleOut percentOutput;

  private InterpolatingDoubleTreeMap shooterAngleLookup = new InterpolatingDoubleTreeMap() { // calculate motorspeed                                                                                            // from  // distance using a // interpolating lookup table.
    {
      put(0.0, 60.3);
      put(1.05, 60.3);
      put(1.27, 56.0);
      put(1.5, 47.0); // TODO: test at wpi
      put(1.73, 45.8);
      put(1.83, 45.0);
      put(2.0, 43.0);
      put(2.3, 40.9);
      put(2.5, 38.5);
      put(2.75, 37.0);
      put(2.9, 35.5);
      put(3.0, 34.7);
      put(3.5, 34.5); // these motorspeeds to meters values are all placeholders, need to actually calculate appropriate motorspeed from corresponding distance;
      put(100.0, 35.0);
      // put(6.0, 30.0);
      // put(7.0, 20.0);
    }
  };

  public ShooterPivot() {
    pivotMotor = new TalonFX(Constants.CANDevices.SHOOTER_PIVOT_ID);
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

    motionMagicConfigs.withMotionMagicAcceleration(degreesToRotation(240.0)); // placeholder, original 36
    motionMagicConfigs.withMotionMagicCruiseVelocity(degreesToRotation(140.0)); // placeholder, original 18, 36/2
    // motionMagicConfigs.withMotionMagicJerk(degreesToRotation(0.0025)); //modifying jerk appears to be a necessary config for motion magic according to MotionMagicVoltage.
    // motionMagicConfigs.withMotionMagicJerk(degreesPerSecondToRotationsPerSecond(0.03));
    // //placeholder

    motorOutputConfig.withInverted(pivotInvert);
    // motorOutputConfig.withDutyCycleNeutralDeadband(0.036);
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
    pivotMotorGains.withKS(kS);
    pivotMotorGains.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
    pivotMotorGains.withGravityType(GravityTypeValue.Arm_Cosine);

    rotationLimits.withForwardSoftLimitEnable(true);
    rotationLimits.withReverseSoftLimitEnable(true);
    rotationLimits.withForwardSoftLimitThreshold(MAXIMUM_LIMIT_ANGLE);
    rotationLimits.withReverseSoftLimitThreshold(MINIMUM_LIMIT_ANGLE);

    pivotMotor.getConfigurator().apply(pivotMotorConfigs);
    pivotMotor.setPosition(STOW_ANGLE);

    // setToStowAngle();
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
   * convert degrees to rotations.
   * 
   * @param degrees
   * @return rotations per second.
   */
  public double degreesToRotation(double degrees) {
    return Units.degreesToRotations(degrees)*GEAR_RATIO;
  }

  /**
   * Control shooterPivot angle with motionMagic.
   * 
   * @param degrees
   */
  public void setPositionDegrees(double degrees) {
    pivotMotor.setControl(motionMagic.withPosition(degreesToRotation(degrees)));
  }

  /** Preset the stow angle of the shooter hood (80 degrees) */
  public void setToStowAngle() {
    pivotMotor.setControl(motionMagic.withPosition(degreesToRotation(STOW_ANGLE)));
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
    return (Units.rotationsToDegrees(pivotMotor.getPosition().getValue()))/GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
