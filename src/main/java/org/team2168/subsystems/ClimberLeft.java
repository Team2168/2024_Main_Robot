// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

import org.team2168.Constants;
import org.team2168.Constants.ClimberMotors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberLeft extends SubsystemBase {

  private TalonFX climberMotorLeft;

  static ClimberLeft instance = null;
  private static final double TIME_UNITS_OF_VELOCITY = 0.1;
  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 0; // placeholder number
  private static final double SPROCKET_RADIUS_IN = 0; // placeholder number
  private static final double INCHES_PER_REV = SPROCKET_RADIUS_IN * 2 * Math.PI;

  private static final double kP = 0;// placeholder
  private static final double kI = 0;// placeholder
  private static final double kD = 0;// placeholder
  //private static final double kF = 0;// placeholder
  private static final double kArbitraryFeedForward = 0;// placeholder 

  private InvertedValue inversion = InvertedValue.Clockwise_Positive;
  private static final double NEUTRAL_DEADBAND = 0.001;

  private static final double kPeakOutput = 1.0;
  private static final double ACCELERATION_LIMIT = inchesToTicks(21.68 * 3.0) * TIME_UNITS_OF_VELOCITY; // placeholder
  private static final double CRUISE_VELOCITY_LIMIT = inchesToTicks(21.68 * 2.5) * TIME_UNITS_OF_VELOCITY; // placeholder

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30; // placeholder 
  private static boolean kSensorPhase = false;

  private static final double CURRENT_LIMIT = 0.0; // it limits when the feature is activited (in amps)
  private static final double THRESHOLD_CURRENT = 0.0; // it tells what the threshold should be for the limit to be activited (in amps)
  private static final boolean CURRENT_LIMIT_ENABLED = true; //placeholder
  private static final double THRESHOLD_TIME = 0.0; // time in seconds of when the limiting should happen after the
                                                    // threshold has been overreached

  private static ElevatorSim climberSimLeft;
  private static final double CARRIAGE_MASS_KG = 4.5;//(placeholder)
  private static final double MIN_HEIGHT_INCHES = -25.0; //+11.9 (30.1 inches is the distance from top of frame to top of moving piece)
  private static final double MAX_HEIGHT_INCHES = 0.5; //placeholder

  //private SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(true, CURRENT_LIMIT, THRESHOLD_CURRENT, THRESHOLD_TIME);


  /** Creates a new Climber. */
  public ClimberLeft() {
    climberMotorLeft = new TalonFX(ClimberMotors.CLIMBER_MOTOR_LEFT);
    climberMotorLeft.getConfigurator().apply(new TalonFXConfiguration());

    var motorConfigs = new MotorOutputConfigs();
    var currentConfigs = new CurrentLimitsConfigs();
    var gains = new Slot0Configs();
    var feedbackConfigs = new FeedbackConfigs();

    motorConfigs.Inverted = inversion;
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);
    motorConfigs.withDutyCycleNeutralDeadband(NEUTRAL_DEADBAND);
    motorConfigs.withPeakForwardDutyCycle(MAX_HEIGHT_INCHES);
    motorConfigs.withPeakReverseDutyCycle(MIN_HEIGHT_INCHES);

    currentConfigs.withSupplyCurrentLimit(CURRENT_LIMIT);
    currentConfigs.withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLED);
    currentConfigs.withSupplyCurrentThreshold(THRESHOLD_CURRENT);
    currentConfigs.withSupplyTimeThreshold(THRESHOLD_TIME);

    //set the gains
    gains.withKP(kP)
    .withKI(kI)
    .withKD(kD);
    //another way to set gains (accessing the member variable directly to change it):
    gains.kP = kP;
    gains.kI = kI;
    gains.kD = kD;
    //climberMotor.config_kF(kPIDLoopIdx, kF, kTimeoutMs);

    /* Feedback Configurations */
    feedbackConfigs.withFeedbackRemoteSensorID(ClimberMotors.FEEDBACK_SENSOR);

    climberSimLeft = new ElevatorSim(
    DCMotor.getFalcon500(1), 
    GEAR_RATIO, 
    CARRIAGE_MASS_KG, 
    Units.inchesToMeters(SPROCKET_RADIUS_IN), 
    Units.inchesToMeters(MIN_HEIGHT_INCHES), 
    Units.inchesToMeters(MAX_HEIGHT_INCHES), 
    kSensorPhase, 
    0, VecBuilder.fill(0.1));

  }

  public static ClimberLeft getInstance() {
    if (instance == null) {
      instance = new ClimberLeft();
    }
    return instance;
  }

  public static double degreesToTicks(double degrees) {
    return (degrees / 360 * TICKS_PER_REV);
  }

  public static double ticksToDegrees(double ticks) {
    return (ticks / TICKS_PER_REV * 360);
  }

  public static double inchesToTicks(double inches) {
    return (inches / INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV;
  }

  public static double ticksToInches(double ticks) {
    return ((ticks / TICKS_PER_REV) / GEAR_RATIO) * INCHES_PER_REV;
  }

  public void setMotorBrake() {
    climberMotorLeft.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setMotorCoast() {
    climberMotorLeft.setNeutralMode(NeutralModeValue.Coast);
  }

   //@Config()
  public void setSpeedVelocity(double speed){
    climberMotorLeft.set(inchesToTicks(speed) * TIME_UNITS_OF_VELOCITY);
  }

  //@Config()
  public void setPosition(double inches){
    //this.position = position;
    climberMotorLeft.setPosition(inchesToTicks(inches), kTimeoutMs);
  }

  /* //@Config()
  public void setPercentOutput(double percentOutput) {
    climberMotor.set(ControlMode.PercentOutput, percentOutput, DemandType.ArbitraryFeedForward, kArbitraryFeedForward);
  }*/

  public void setToZero(){
    climberMotorLeft.setPosition(0, kTimeoutMs);
  }

  public void setInvertPosition(boolean invert){
    climberMotorLeft.setInverted(invert);
  }

  public void setPercentOutput(double percentOutput) {
    climberMotorLeft.setVoltage(percentOutput);
  }

  //@Log(name = "placeholder", rowIndex = 0, columnIndex = 0)
  public double getCurrentSpeed(){
    return climberMotorLeft.get();
  }

  //@Log(name = "placeholder", rowIndex = 0, columnIndex = 0)
  public double getspeedVelocity(){
    return (ticksToInches(climberMotorLeft.get()) / TIME_UNITS_OF_VELOCITY);
  }

  //@Log(name = "placeholder", rowIndex = 0, columnIndex = 0)
  public double getPosition(){
    return ticksToInches(climberMotorLeft.get());
  }

  //@Log(name = "placeholder", rowIndex = 0, columnIndex = 0)
  public boolean getInvertPosition(){
    return climberMotorLeft.getInverted();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {

    /* 
    // This method will be called once per scheduler run during simulation
     // Affect motor outputs by main system battery voltage dip 
    climberMotorSim.setBusVoltage(RobotController.getBatteryVoltage());

    // Pass motor output voltage to physics sim
    climberSim.setInput(climberMotorSim.getMotorOutputLeadVoltage());
    climberSim.update(Constants.ClimberMotors.UPDATE_TIME);

    // System.out.println("Climber pos: " + climberSim.getPositionMeters());

    // Update motor sensor states based on physics model
    double sim_velocity_ticks_per_100_ms = inchesToTicks(Units.metersToInches(climberSim.getVelocityMetersPerSecond())) * TIME_UNITS_OF_VELOCITY;
    double sim_position = inchesToTicks(Units.metersToInches(climberSim.getPositionMeters()));
    climberMotorSim.setIntegratedSensorRawPosition((int) sim_position);
    climberMotorSim.setIntegratedSensorVelocity((int) sim_velocity_ticks_per_100_ms);

    */

    //This is some stuff we were testing out. This may not be in the final code at all.
  }

  
}
