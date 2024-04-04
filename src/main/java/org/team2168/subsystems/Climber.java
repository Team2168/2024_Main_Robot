// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.team2168.Constants.ClimberMotors;

public class Climber extends SubsystemBase implements Loggable {

  private CANSparkMax climberMotor = new CANSparkMax(ClimberMotors.CLIMBER_MOTOR, MotorType.kBrushed);
  // private RelativeEncoder climberEncoder = climberMotor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, TICKS_PER_REV);

  static Climber instance = null;

  public static final int TICKS_PER_REV = 50;
  private static final double TIME_UNITS_OF_VELOCITY = 1; //this might need to be changed later
  private static final double GEAR_RATIO = 100.0; 
  private static final double MOTOR_DIAMETER_IN = 1.73228; 
  private static final double INCHES_PER_REV = MOTOR_DIAMETER_IN * Math.PI;

  private SparkPIDController m_pidController;

  private static final double kMaxOutput = 1.0;// placeholder
  private static final double kMinOutput = -1.0;// placeholder
  private static final double kMaxVel= inchesToRotations(2.5) * TIME_UNITS_OF_VELOCITY;; //placeholder
  private static final double kMaxAcc= inchesToRotations(3.0) * TIME_UNITS_OF_VELOCITY;; //placeholder

  private static final double kP = 0.5;// placeholder
  private static final double kI = 0;// placeholder
  private static final double kD = 0;// placeholder
  //private static final double kF = 0;// placeholder
  private static final double kArbitraryFeedForward = 0;// placeholder 

  private static final double NEUTRAL_DEADBAND = 0.001;

  private static final int kPIDLoopIdx = 0;
  private static boolean kSensorPhase = false;

  //private SparkLimitSwitch forwardLimit;
  //private SparkLimitSwitch reverseLimit;

  private static final int CURRENT_LIMIT = 25; // it limits when the feature is activited (in amps)
  private static final int FREE_LIMIT = 30; // it tells what the threshold should be for the limit to be activited (in amps)

  private static final double CARRIAGE_MASS_KG = 4.5;//(placeholder)
  private static final double MIN_HEIGHT_INCHES = -25.0; //
  private static final double MAX_HEIGHT_INCHES = 0.5; //placeholder

  //private SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(true, CURRENT_LIMIT, THRESHOLD_CURRENT, THRESHOLD_TIME);

  /** Creates a new Climber. */
  public Climber() {
    m_pidController = climberMotor.getPIDController();
    //m_Encoder = climberMotor.getEncoder(SparkRelativeEncoder.Type.kNoSensor, 50); // Encoder object created to display position values

    // m_pidController.setFeedbackDevice(climberEncoder);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput); 

    m_pidController.setSmartMotionMaxVelocity(kMaxVel, 0);
    m_pidController.setSmartMotionMaxAccel(kMaxAcc, 0);

    //sets PID gains
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);

    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    climberMotor.setSmartCurrentLimit(CURRENT_LIMIT, FREE_LIMIT);
    climberMotor.setIdleMode(IdleMode.kBrake);
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

    /** Sets the velocity of where the climber should be
   * @param velocity - velocity (in inches per second)
   * @return sets the reference for the motor controller that sets the velocity of the climber motor
   */
  public void setSpeedVelocity(double velocity){
    m_pidController.setReference(inchesToRotations(velocity) * TIME_UNITS_OF_VELOCITY, ControlType.kVelocity, 0, kArbitraryFeedForward);
  }

  /**
   * Sets the speed of the climber
   * @param speed speed of the climber (-1.0 to 1.0)
   */
  public void setSpeed(double speed) {
    climberMotor.set(speed);
  }

    /** Sets the position of where the climber should be
   * @param in - inches
   * @return sets the reference for the motor controller that sets the position using smart motion 
   */
  public void setPosition(double in){
    m_pidController.setReference(inchesToTicks(in), ControlType.kSmartMotion, 0, kArbitraryFeedForward);
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

  // @Log(name = "Speed Velocity", rowIndex = 0, columnIndex = 1)
  // public double getSpeedVelocity(){
  //   return (rotationsToInches(climberEncoder.getVelocity()) / 60); 
  // }

  // @Log(name = "Position in inches", rowIndex = 0, columnIndex = 2)
  // public double getPositionInches(){
  //   return degreesToInches(Units.rotationsToDegrees(climberEncoder.getPosition()));
  // }

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
