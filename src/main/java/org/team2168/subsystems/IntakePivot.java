// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import org.team2168.Constants.CANDevices;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;

public class IntakePivot extends SubsystemBase {

  private static TalonFX intakePivotOne = new TalonFX(CANDevices.intakePivotL); // leader motor
  private static TalonFX intakePivotTwo = new TalonFX(CANDevices.intakePivotR); // follower motor
  private static IntakePivot instance = null;

  public static IntakePivot getInstance() {
    if(instance == null)
    instance = new IntakePivot();
    return instance;
  }

  private static InvertedValue intakeInvertOne = InvertedValue.Clockwise_Positive; //TODO: check value
  private static InvertedValue intakeInvertTwo = InvertedValue.CounterClockwise_Positive;

 // private static int intakeTimeoutMs = 30;
 // private static double peakOutput = 1.0;
  private final boolean ENABLE_CURRENT_LIMIT = true;
  private final double CONTINUOUS_CURRENT_LIMIT = 15.0;
  private final double TRIGGER_THRESHOLD_LIMIT = 20;
  private final double TRIGGER_THRESHOLD_TIME = 0.2;
 // private final double minuteInHundredMs = 600.0;
  private double neutralDeadband = 0.01;
  private double maxForwardOutput = 1;
  private double maxBackwardOutput = -1;
  final double MIN_ANGLE = -120;
  final double MAX_ANGLE = 0;
  private double motionMagicAcceleration = degreesToRot(500.0);
  private double motionMagicCruiseVelocity = degreesToRot(250.0);
  private double kV = 0.12;
  private double kA = 0.1;
  private double sensorOffset = degreesToRot(-120.0);
  final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0); // TODO: change maybe to sensor offset

  private final double TICKS_PER_REV = 2048;
  private final static double GEAR_RATIO = 50.0; // updated after waterbury

  private double kP = 15.0;
  private double kI = 0.0;
  private double kD = 0.3;
  private double kG = -1.3;
  private GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;



  private IntakePivot() {
    intakePivotOne.clearStickyFaults();
    intakePivotTwo.clearStickyFaults();
    intakePivotOne.getConfigurator().apply(new TalonFXConfiguration()); //resets leader motor to its factory default
    intakePivotTwo.getConfigurator().apply(new TalonFXConfiguration()); //resets follower motor to its factory default

    var currentConfigs = new CurrentLimitsConfigs();
    var PIDconfigs = new SlotConfigs();
    var motionMagicConfigs = new MotionMagicConfigs();
    var motorOneConfigs = new MotorOutputConfigs();
    var motorTwoConfigs = new MotorOutputConfigs();
    var softLimitsConfigs = new SoftwareLimitSwitchConfigs();

    motorOneConfigs.withInverted(intakeInvertOne);
    //motorConfigs.withInverted(intakeInvertTwo);
    motorOneConfigs.withDutyCycleNeutralDeadband(neutralDeadband);
    motorOneConfigs.withPeakForwardDutyCycle(maxForwardOutput);
    motorOneConfigs.withPeakReverseDutyCycle(maxBackwardOutput);

    motorTwoConfigs.withInverted(intakeInvertTwo);
    motorTwoConfigs.withDutyCycleNeutralDeadband(neutralDeadband);
    motorTwoConfigs.withPeakForwardDutyCycle(maxForwardOutput);
    motorTwoConfigs.withPeakReverseDutyCycle(maxBackwardOutput);


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

    softLimitsConfigs
      .withForwardSoftLimitThreshold(degreesToRot(5.0)) // extra five degrees for error tolerance
      .withForwardSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(degreesToRot(-125.0)) // extra five degrees for error tolerance
      .withReverseSoftLimitEnable(true);

    var intakeRaiseAndLowerOne = intakePivotOne.getConfigurator();
    var intakeRaiseAndLowerTwo = intakePivotTwo.getConfigurator();
    
    intakeRaiseAndLowerOne.apply(motorOneConfigs, 0.2);
    intakeRaiseAndLowerOne.apply(currentConfigs, 0.2);
    intakeRaiseAndLowerOne.apply(PIDconfigs, 0.2);
    intakeRaiseAndLowerOne.apply(softLimitsConfigs, 0.2);
    intakeRaiseAndLowerOne.apply(motionMagicConfigs, 0.2);

    intakeRaiseAndLowerTwo.apply(motorTwoConfigs, 0.2);
    intakeRaiseAndLowerTwo.apply(currentConfigs, 0.2);
    intakeRaiseAndLowerTwo.apply(motionMagicConfigs, 0.2);
    //intakeRaiseAndLowerTwo.apply(softLimitsConfigs);
    intakeRaiseAndLowerTwo.apply(PIDconfigs, 0.2);

    intakePivotOne.setNeutralMode(NeutralModeValue.Brake);
    intakePivotTwo.setNeutralMode(NeutralModeValue.Brake);

    // intakePivotOne.setPosition(sensorOffset, 0.2);
    // intakePivotTwo.setPosition(sensorOffset, 0.2); // reset only upon robot init
    
    //sets the same settings to the motor intakePivotTwo from intakePivotOne
    //intakePivotTwo.setControl(new Follower(intakePivotOne.getDeviceID(), true));
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
    intakePivotOne.setControl(motionMagicVoltage.withPosition((degreesToRot(demand))));
    intakePivotTwo.setControl(motionMagicVoltage.withPosition((degreesToRot(demand))));
    //intakePivotTwo.setControl(new Follower(intakePivotOne.getDeviceID(), true));
  }

  public boolean getIntakeOneInvert() {
    return intakePivotOne.getInverted(); // clockwise positive is true
  }

  public boolean getIntakeTwoInvert() {
    return intakePivotTwo.getInverted(); // clockwise positive is true
  }

  public void setSpeed(double percentOutput) {
    intakePivotOne.set(percentOutput);
    //intakePivotTwo.set(percentOutput);
  }

  public void resetIntakeEncodersToStow() {
    intakePivotOne.setPosition(sensorOffset, 0.2);
    intakePivotTwo.setPosition(sensorOffset, 0.2);
  }

  public void setInvertConfiguration() {
    var motorOneConfigs = new MotorOutputConfigs();
    var motorTwoConfigs = new MotorOutputConfigs();

    motorOneConfigs.withInverted(intakeInvertOne);
    //motorConfigs.withInverted(intakeInvertTwo);
    motorOneConfigs.withDutyCycleNeutralDeadband(neutralDeadband);
    motorOneConfigs.withPeakForwardDutyCycle(maxForwardOutput);
    motorOneConfigs.withPeakReverseDutyCycle(maxBackwardOutput);

    motorTwoConfigs.withInverted(intakeInvertTwo);
    motorTwoConfigs.withDutyCycleNeutralDeadband(neutralDeadband);
    motorTwoConfigs.withPeakForwardDutyCycle(maxForwardOutput);
    motorTwoConfigs.withPeakReverseDutyCycle(maxBackwardOutput);

    intakePivotOne.getConfigurator().apply(motorOneConfigs, 0.2);
    intakePivotTwo.getConfigurator().apply(motorTwoConfigs, 0.2);
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
