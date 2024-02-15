// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class IntakePivot extends SubsystemBase {

  private static TalonFX intakePivotOne = new TalonFX(0); // leader motor
  private static TalonFX intakePivotTwo = new TalonFX(0); // follower motor
  private static IntakePivot instance = null;

  public IntakePivot getInstance() {
    if(instance == null)
    instance = new IntakePivot();
    return instance;
  }

  private static InvertedValue intakeInvertOne = InvertedValue.Clockwise_Positive; //TODO: check value
 // private static InvertedValue intakeInvertTwo = InvertedValue.CounterClockwise_Positive;

 // private static int intakeTimeoutMs = 30;
 // private static double peakOutput = 1.0;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 20.0;
  private final double TRIGGER_THRESHOLD_LIMIT = 25;
  private final double TRIGGER_THRESHOLD_TIME = 0.2;
 // private final double minuteInHundredMs = 600.0;
  private double neutralDeadband = 0.05;
  private double maxForwardOutput = 1;
  private double maxBackwardOutput = -1;
  final double MIN_ANGLE = 0;
  final double MAX_ANGLE = 120;
  private double motionMagicAcceleration = 5.0;
  private double motionMagicCruiseVelocity = 10.0;
  private double kV = 0.12;
  private double kA = 0.1;
  final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrent = new MotionMagicTorqueCurrentFOC(0); // TODO: change maybe to sensor offset
  private double sensorOffset = degreesToRot(-120);

  private final double TICKS_PER_REV = 2048;
  private final static double GEAR_RATIO = 28.14;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kG = 14;
  private GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;



  private IntakePivot() {
    intakePivotOne.getConfigurator().apply(new TalonFXConfiguration()); //resets leader motor to its factory default
    intakePivotTwo.getConfigurator().apply(new TalonFXConfiguration()); //resets follower motor to its factory default

    var currentConfigs = new CurrentLimitsConfigs();
    var PIDconfigs = new SlotConfigs();
    var motionMagicConfigs = new MotionMagicConfigs();
    var leaderMotorConfigs = new MotorOutputConfigs();
    var followerMotorConfigs = new MotorOutputConfigs();

    leaderMotorConfigs.withInverted(intakeInvertOne);
    leaderMotorConfigs.withDutyCycleNeutralDeadband(neutralDeadband);
    leaderMotorConfigs.withPeakForwardDutyCycle(maxForwardOutput);
    leaderMotorConfigs.withPeakReverseDutyCycle(maxBackwardOutput);

   // followerMotorConfigs.withInverted(intakeInvertTwo);

    currentConfigs
      .withSupplyCurrentLimitEnable(ENABLE_CURRENT_LIMIT)
      .withSupplyCurrentLimit(CONTINUOUS_CURRENT_LIMIT)
      .withSupplyCurrentThreshold(TRIGGER_THRESHOLD_LIMIT)
      .withSupplyTimeThreshold(TRIGGER_THRESHOLD_TIME);
    //intakePivotConfig.withCurrentLimits(currentLimitConfig);
    //intakePivotConfig.supplyCurrLimit = talonCurrentLimit;

    PIDconfigs
      .withKP(kP)
      .withKI(kI)
      .withKD(kD)
      .withKG(kG)
      .withGravityType(gravityType);

    motionMagicConfigs
      .withMotionMagicAcceleration(motionMagicAcceleration)
      .withMotionMagicCruiseVelocity(motionMagicCruiseVelocity)
      .withMotionMagicExpo_kA(kA)
      .withMotionMagicExpo_kV(kV);

    var intakeRaiseAndLowerOne = intakePivotOne.getConfigurator();
    var intakeRaiseAndLowerTwo = intakePivotTwo.getConfigurator();
    
    intakeRaiseAndLowerOne.apply(leaderMotorConfigs);
    intakeRaiseAndLowerOne.apply(currentConfigs);
    intakeRaiseAndLowerOne.apply(PIDconfigs);

    intakeRaiseAndLowerTwo.apply(followerMotorConfigs);
    intakeRaiseAndLowerTwo.apply(currentConfigs);
    // intakeRaiseAndLower.apply(motionMagicConfigs);

    intakePivotOne.setNeutralMode(NeutralModeValue.Coast);

    intakePivotOne.setPosition(sensorOffset);
    
    //sets the same settings to the motor intakePivotTwo from intakePivotOne
    intakePivotTwo.setControl(new Follower(intakePivotOne.getDeviceID(), true));
  }

  /**
   * converts rotation to degrees
   * @param ticks amount of rotations
   * @return amount degrees from amount of rotations
   */
  public static double rotToDegrees(double rot) {
    return (rot) / GEAR_RATIO * 360.0;
  }

  /**
   * converts degrees to rotations
   * @param degrees amount of degrees/angles to move intakepivot up
   * @return amount of degrees from amount of rotations
   */
  public static double degreesToRot(double degrees) {
    return (degrees/360) * GEAR_RATIO;
  }

  /**
   * converts degrees to ticks
   * @param degrees amount of degrees
   * @return amount of ticks from amount degrees
   */
  public double degreesToTicks(double degrees) {
    return (degrees/360.0) * GEAR_RATIO * TICKS_PER_REV;
  }

  /**
   * sets intake position using motion magic torque current
   * @param degrees amount of degrees of position
   */
  public void setIntakePivotPosition(double degrees) {
    var demand = MathUtil.clamp(degrees, MIN_ANGLE, MAX_ANGLE);
    intakePivotOne.setControl(motionMagicTorqueCurrent.withPosition((degreesToRot(demand))));
  }

  @Log(name = "Position (deg)", rowIndex = 0, columnIndex = 0)

  /**
   * gets the positon of the intake
   * @return the position of intake in degrees
   */
  public double getIntakePivotPosition() {
    return rotToDegrees(intakePivotOne.getPosition().getValueAsDouble());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
