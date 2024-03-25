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
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import java.beans.Encoder;
import java.security.CodeSource;

import org.team2168.Constants;
import org.team2168.Constants.ClimberMotors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {

  private CANSparkMax climberMotor;
  private RelativeEncoder climberEncoder;

  static Climber instance = null;
  public static final double TICKS_PER_REV = 50.0;
  private static final double TIME_UNITS_OF_VELOCITY = 1; //this might need to be changed later
  private static final double GEAR_RATIO = 79.0; 
  private static final double MOTOR_DIAMETER_IN = 1.73228; 
  private static final double INCHES_PER_REV = MOTOR_DIAMETER_IN * Math.PI;

  private SparkPIDController m_pidController;
  private RelativeEncoder m_Encoder;
  private static final double kMaxOutput = 1;// placeholder
  private static final double kMinOutput = -1;// placeholder
  private static final double kMaxVel= inchesToRotations(21.68 * 2.5) * TIME_UNITS_OF_VELOCITY;; //placeholder
  private static final double kMaxAcc= inchesToRotations(21.68 * 3.0) * TIME_UNITS_OF_VELOCITY;; //placeholder

  private static final double kP = 0;// placeholder
  private static final double kI = 0;// placeholder
  private static final double kD = 0;// placeholder
  //private static final double kF = 0;// placeholder
  private static final double kArbitraryFeedForward = 0;// placeholder 

  private static final double NEUTRAL_DEADBAND = 0.001;

  private static final int kPIDLoopIdx = 0;
  private static boolean kSensorPhase = false;

  //private SparkLimitSwitch forwardLimit;
  //private SparkLimitSwitch reverseLimit;

 private static final int CURRENT_LIMIT = 0; // it limits when the feature is activited (in amps)
private static final int FREE_LIMIT = 0; // it tells what the threshold should be for the limit to be activited (in amps)
  private static final boolean CURRENT_LIMIT_ENABLED = true; //placeholder
  private static final double THRESHOLD_TIME = 0.0; // time in seconds of when the limiting should happen after the
                                                    // threshold has been overreached
  private static final double CARRIAGE_MASS_KG = 4.5;//(placeholder)
  private static final double MIN_HEIGHT_INCHES = -25.0; //+11.9 (30.1 inches is the distance from top of frame to top of moving piece)
  private static final double MAX_HEIGHT_INCHES = 0.5; //placeholder

  //private SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(true, CURRENT_LIMIT, THRESHOLD_CURRENT, THRESHOLD_TIME);


  /** Creates a new Climber. */
  public Climber() {
    climberMotor = new CANSparkMax(ClimberMotors.CLIMBER_MOTOR, MotorType.kBrushed);
    climberEncoder = climberMotor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 50);

    m_pidController = climberMotor.getPIDController();
    m_Encoder = climberMotor.getEncoder(); // Encoder object created to display position values

    m_pidController.setFeedbackDevice(m_Encoder);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput); 

    m_pidController.setSmartMotionMaxVelocity(kMaxVel, 0);
    m_pidController.setSmartMotionMaxAccel(kMaxAcc, 0);

    //sets PID gains
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    climberMotor.setSmartCurrentLimit(CURRENT_LIMIT, FREE_LIMIT);
    //currentConfigs.withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLED);
    //currentConfigs.withSupplyTimeThreshold(THRESHOLD_TIME);
  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  /** Converts degrees to rotations
   * @param degrees
   * @return returns the converted result of the inputed degrees converted to rotations
   */
  public static double degreesToRotations(double degrees) {
    return (degrees / 360);
  }

    /** Converts rotations to degrees 
   * @param rotations
   * @return returns the converted result of the inputed rotations to degrees
   */
  public static double rotationsToDegrees(double rotations) {
    return (rotations * 360);
  }

    /** Converts inches to rotations
   * @param inches
   * @return returns the converted result of the inputed inches converted to rotations
   */
  public static double inchesToRotations(double inches) {
    return inches / INCHES_PER_REV;
  }

    /** Converts rotations to inches
   * @param rotations
   * @return returns the converted result of the inputed rotations converted to inches
   */
  public static double rotationsToInches(double rotations){
    return rotations * INCHES_PER_REV;
  }

  /** Converts degrees to inches
   * @param degrees
   * @return returns the converted result of the inputed degrees to inches
   */
  public static double degreesToInches(double degrees){
    return INCHES_PER_REV * (degreesToRotations(degrees));
  }

  private static double inchesToTicks(double inches) {
    return (inches / INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV;
  }

  private static double ticksToInches(double ticks) {
    return (ticks / TICKS_PER_REV) / GEAR_RATIO * INCHES_PER_REV;
  }

  public void setMotorBrake() {
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setMotorCoast() {
    climberMotor.setIdleMode(IdleMode.kCoast);
  }

    /** Sets the position of where the climber should be
   * @param velocity - velocity (in inches)
   * @return sets the reference for the motor controller that sets the velocity of the climber motor
   */
  public void setSpeedVelocity(double velocity){
    m_pidController.setReference(inchesToRotations(velocity) * TIME_UNITS_OF_VELOCITY, ControlType.kVelocity, 0, kArbitraryFeedForward);
  }

    /** Sets the position of where the climber should be
   * @param in - inches
   * @return sets the reference for the motor controller that sets the position using smart motion 
   */
  public void setPosition(double in){
    m_pidController.setReference(inchesToRotations(in) * TIME_UNITS_OF_VELOCITY, ControlType.kSmartMotion, 0, kArbitraryFeedForward);
  }

  public void setToZero(){
    m_pidController.setReference(0, ControlType.kSmartMotion, 0, kArbitraryFeedForward);
  }
  
  /** Inverts the climber's position (where does it go)
   * @param invert - true of false of wheter it inverts the motor movement or not
   * @return sets the inversion of the climber (is it inverted or not?)
   */
  public void setInvertPosition(boolean invert){
    climberMotor.setInverted(invert);
  }

   /** Functions the same as "SetVolt()" expect it isn't a set and forget method
   * @param speed - the voltage input
   * @return sets voltage
   */
  public void setPercentOutput(double speed){
    m_pidController.setReference(speed, ControlType.kVoltage,0, kArbitraryFeedForward); //functions the same as "SetVolt()" expect it isn't a set and forget method
  }
 /** Sets the climber's voltage
   * @param volt - input of voltage
   * @return sets voltage
   */
  public void setVolt(double volt) {
    climberMotor.setVoltage(volt);
  }

  @Log(name = "Current Set Speed", rowIndex = 0, columnIndex = 0)
  public double getCurrentSetSpeed(){
    return climberMotor.get();
  }

  @Log(name = "Speed Velocity", rowIndex = 0, columnIndex = 1)
  public double getSpeedVelocity(){
    return (rotationsToInches(m_Encoder.getVelocity()) / 60); 
  }

  @Log(name = "Position in inches", rowIndex = 0, columnIndex = 2)
  public double getPositionInches(){
    return degreesToInches(Units.rotationsToDegrees(m_Encoder.getPosition()));
  }

  @Log(name = "Voltage", rowIndex = 1, columnIndex = 4)
  public double getVoltage(){
    return climberMotor.getBusVoltage();
  }

  @Log(name = "Invert Position", rowIndex = 0, columnIndex = 3)
  public boolean getInvertPosition(){
    return climberMotor.getInverted();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {

  }

  
}
