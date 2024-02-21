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
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import org.team2168.Constants;
import org.team2168.Constants.ClimberMotors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Climber extends SubsystemBase {

  private CANSparkMax climberMotorLeft;
  private CANSparkMax climberMotorRight;

  static Climber instance = null;
  private static final double TIME_UNITS_OF_VELOCITY = 1; //this might need to be changed later
  private static final double GEAR_RATIO = 79; 
  private static final double MOTOR_DIAMETER_IN = 1.73228; 
  private static final double INCHES_PER_REV = MOTOR_DIAMETER_IN * Math.PI;

  private SparkPIDController m_leftpidController;
  private SparkPIDController m_rightpidController;
  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;
  private static final double kMaxOutput = 1;// placeholder
  private static final double kMinOutput = 0.1;// placeholder
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
  private static ElevatorSim climberSimLeft;
  private static final double CARRIAGE_MASS_KG = 4.5;//(placeholder)
  private static final double MIN_HEIGHT_INCHES = -25.0; //+11.9 (30.1 inches is the distance from top of frame to top of moving piece)
  private static final double MAX_HEIGHT_INCHES = 0.5; //placeholder

  //private SupplyCurrentLimitConfiguration talonCurrentLimit = new SupplyCurrentLimitConfiguration(true, CURRENT_LIMIT, THRESHOLD_CURRENT, THRESHOLD_TIME);


  /** Creates a new Climber. */
  public Climber() {
    climberMotorLeft = new CANSparkMax(ClimberMotors.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
    climberMotorRight = new CANSparkMax(ClimberMotors.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);

    m_leftpidController = climberMotorLeft.getPIDController();
    m_leftEncoder = climberMotorLeft.getEncoder(); // Encoder object created to display position values
    m_rightpidController = climberMotorRight.getPIDController();
    m_rightEncoder = climberMotorRight.getEncoder(); // Encoder object created to display position values

    m_leftpidController.setFeedbackDevice(m_leftEncoder);
    m_leftpidController.setOutputRange(kMinOutput, kMaxOutput); 

    m_leftpidController.setSmartMotionMaxVelocity(kMaxVel, 0);
    m_leftpidController.setSmartMotionMaxAccel(kMaxAcc, 0);

    //sets PID gains
    m_leftpidController.setP(kP);
    m_leftpidController.setI(kI);
    m_leftpidController.setD(kD);
    m_leftpidController.setOutputRange(kMinOutput, kMaxOutput);

    climberMotorLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climberMotorLeft.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    climberMotorLeft.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    climberMotorLeft.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    climberMotorLeft.setSmartCurrentLimit(CURRENT_LIMIT, FREE_LIMIT);

    m_rightpidController.setFeedbackDevice(m_rightEncoder);
    m_rightpidController.setOutputRange(kMinOutput, kMaxOutput); 

    m_rightpidController.setSmartMotionMaxVelocity(kMaxVel, 0);
    m_rightpidController.setSmartMotionMaxAccel(kMaxAcc, 0);

    //sets PID gains
    m_rightpidController.setP(kP);
    m_rightpidController.setI(kI);
    m_rightpidController.setD(kD);
    m_rightpidController.setOutputRange(kMinOutput, kMaxOutput);

    climberMotorRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climberMotorRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    climberMotorRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    climberMotorRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
    
    climberMotorRight.setSmartCurrentLimit(CURRENT_LIMIT, FREE_LIMIT);
    //currentConfigs.withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLED);
    //currentConfigs.withSupplyTimeThreshold(THRESHOLD_TIME);

    climberSimLeft = new ElevatorSim(
    DCMotor.getFalcon500(1), 
    GEAR_RATIO, 
    CARRIAGE_MASS_KG, 
    Units.inchesToMeters(MOTOR_DIAMETER_IN), 
    Units.inchesToMeters(MIN_HEIGHT_INCHES), 
    Units.inchesToMeters(MAX_HEIGHT_INCHES), 
    kSensorPhase, 
    0, VecBuilder.fill(0.1));

  }

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  public static double degreesToRotations(double degrees) {
    return (degrees / 360);
  }

  public static double rotationsToDegrees(double rotations) {
    return (rotations * 360);
  }

  public static double inchesToRotations(double inches) {
    return inches / INCHES_PER_REV;
  }

  public static double rotationsToInches(double rotations){
    return rotations * INCHES_PER_REV;
  }

  public static double degreesToInches(double degrees){
    return INCHES_PER_REV * (degreesToRotations(degrees));
  }

  public void setLeftMotorBrake() {
    climberMotorLeft.setIdleMode(IdleMode.kBrake);
  }

  public void setLeftMotorCoast() {
    climberMotorLeft.setIdleMode(IdleMode.kCoast);
  }

  //@Config()
  public void setLeftSpeedVelocity(double velocity){
    m_leftpidController.setReference(inchesToRotations(velocity) * TIME_UNITS_OF_VELOCITY, ControlType.kVelocity, 0, kArbitraryFeedForward);
  }

  //@Config()
  public void setLeftPosition(double in){
    m_leftpidController.setReference(inchesToRotations(in) * TIME_UNITS_OF_VELOCITY, ControlType.kSmartMotion, 0, kArbitraryFeedForward);
  }

  public void setLeftToZero(){
    m_leftpidController.setReference(0, ControlType.kSmartMotion, 0, kArbitraryFeedForward);
  }

  public void setLeftInvertPosition(boolean invert){
    climberMotorLeft.setInverted(invert);
  }

  public void setLeftPercentOutput(double speed){
    m_leftpidController.setReference(speed, ControlType.kVoltage,0, kArbitraryFeedForward); //functions the same as "SetVolt()" expect it isn't a set and forget method
  }

  public void setLeftVolt(double volt) {
    climberMotorLeft.setVoltage(volt);
  }

  public void setRightMotorBrake() {
    climberMotorRight.setIdleMode(IdleMode.kBrake);
  }

  public void setRightMotorCoast() {
    climberMotorRight.setIdleMode(IdleMode.kCoast);
  }

    public void setRightSpeedVelocity(double velocity){
    m_rightpidController.setReference(inchesToRotations(velocity) * TIME_UNITS_OF_VELOCITY, ControlType.kVelocity, 0, kArbitraryFeedForward);
  }

  //@Config()
  public void setRightPosition(double in){
    m_rightpidController.setReference(inchesToRotations(in) * TIME_UNITS_OF_VELOCITY, ControlType.kSmartMotion, 0, kArbitraryFeedForward);
  }

  public void setRightPercentOutput(double speed){
    m_rightpidController.setReference(speed, ControlType.kVoltage,0, kArbitraryFeedForward); //functions the same as "SetVolt()" expect it isn't a set and forget method
  }

  public void setRightToZero(){
    m_rightpidController.setReference(0, ControlType.kSmartMotion, 0, kArbitraryFeedForward);
  }

  public void setRightInvertPosition(boolean invert){
    climberMotorRight.setInverted(invert);
  }

  public void setRightVolt(double volt) {
    climberMotorRight.setVoltage(volt);
  }

  @Log(name = "Current Set Speed", rowIndex = 0, columnIndex = 0)
  public double getLeftCurrentSetSpeed(){
    return climberMotorLeft.get();
  }

  @Log(name = "Speed Velocity", rowIndex = 0, columnIndex = 1)
  public double getLeftspeedVelocity(){
    return (rotationsToInches(m_leftEncoder.getVelocity()) / 60); 
  }

  @Log(name = "Position in inches", rowIndex = 0, columnIndex = 2)
  public double getLeftPositionInches(){
    return degreesToInches(Units.rotationsToDegrees(m_leftEncoder.getPosition()));
  }

  @Log(name = "Voltage", rowIndex = 1, columnIndex = 4)
  public double getLeftVoltage(){
    return climberMotorLeft.getBusVoltage();
  }

  @Log(name = "Invert Position", rowIndex = 0, columnIndex = 3)
  public boolean getLeftInvertPosition(){
    return climberMotorLeft.getInverted();
  }

   @Log(name = "Current Set Speed", rowIndex = 1, columnIndex = 0)
  public double getRightCurrentSetSpeed(){
    return climberMotorRight.get();
  }

  @Log(name = "Speed Velocity", rowIndex = 1, columnIndex = 1)
  public double getRightspeedVelocity(){
    return (rotationsToInches(m_rightEncoder.getVelocity()) / 60); 
  }

  @Log(name = "Position in inches", rowIndex = 1, columnIndex = 2)
  public double getRightPositionInches(){
    return degreesToInches(Units.rotationsToDegrees(m_rightEncoder.getPosition()));
  }

  @Log(name = "Voltage", rowIndex = 1, columnIndex = 4)
  public double getRightVoltage(){
    return climberMotorRight.getBusVoltage();
  }

  @Log(name = "Invert Position", rowIndex = 1, columnIndex = 3)
  public boolean getRightInvertPosition(){
    return climberMotorRight.getInverted();
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
