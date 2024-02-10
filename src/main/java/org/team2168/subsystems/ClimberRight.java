// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import org.team2168.subsystems.ClimberLeft;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ClimberRight extends SubsystemBase {
  /** Creates a new ClimberRight. */
  private CANSparkMax climberMotorRight;

  static ClimberRight instance = null;
  private static final double TIME_UNITS_OF_VELOCITY = 0.1;
  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 0; // placeholder number
  private static final double SPROCKET_RADIUS_IN = 0; // placeholder number
  private static final double INCHES_PER_REV = SPROCKET_RADIUS_IN * 2 * Math.PI;

    private SparkPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private static final double kMaxOutput = 0;// placeholder
  private static final double kMinOutput = 0;// placeholder
  private static final double kMaxVel= ClimberLeft.inchesToTicks(21.68 * 2.5) * TIME_UNITS_OF_VELOCITY;; //placeholder
  private static final double kMaxAcc= ClimberLeft.inchesToTicks(21.68 * 3.0) * TIME_UNITS_OF_VELOCITY;; //placeholder
  
  private static final double kP = 0;// placeholder
  private static final double kI = 0;// placeholder
  private static final double kD = 0;// placeholder
  //private static final double kF = 0;// placeholder
  private static final double kArbitraryFeedForward = 0;// placeholder 

  private InvertedValue inversion = InvertedValue.Clockwise_Positive; //might have to be set as negative
  private static final double NEUTRAL_DEADBAND = 0.001;

  private static final double kPeakOutput = 1.0;

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30; // placeholder 
  private static boolean kSensorPhase = false;

  public String kEnable;
  public String kDisable;

  private static final int CURRENT_LIMIT = 0; // it limits when the feature is activited (in amps)
  private static final int FREE_LIMIT = 0; // it tells what the threshold should be for the limit to be activited (in amps)
  private static final boolean CURRENT_LIMIT_ENABLED = true; //placeholder
  private static final double THRESHOLD_TIME = 0.0; // time in seconds of when the limiting should happen after the
                                                    // threshold has been overreached

  private static ElevatorSim climberSimRight;
  private static final double CARRIAGE_MASS_KG = 4.5;//(placeholder)
  private static final double MIN_HEIGHT_INCHES = -25.0; //+11.9 (30.1 inches is the distance from top of frame to top of moving piece)
  private static final double MAX_HEIGHT_INCHES = 0.5; //placeholder

  public ClimberRight() {
    climberMotorRight = new CANSparkMax(ClimberMotors.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);

    m_pidController = climberMotorRight.getPIDController();
    m_pidController.setFeedbackDevice(m_encoder);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput); 
   
    // Encoder object created to display position values
    m_encoder = climberMotorRight.getEncoder();

    m_pidController.setSmartMotionMaxVelocity(kMaxVel, 0);
    m_pidController.setSmartMotionMaxAccel(kMaxAcc, 0);

    //sets PID gains
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    climberMotorRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climberMotorRight.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    climberMotorRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 15);
    climberMotorRight.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    climberMotorRight.setSmartCurrentLimit(CURRENT_LIMIT, FREE_LIMIT);
    //currentConfigs.withSupplyCurrentLimitEnable(CURRENT_LIMIT_ENABLED);
    //currentConfigs.withSupplyTimeThreshold(THRESHOLD_TIME);
    
    climberSimRight = new ElevatorSim(
      DCMotor.getFalcon500(1), 
      GEAR_RATIO, 
      CARRIAGE_MASS_KG, 
      Units.inchesToMeters(SPROCKET_RADIUS_IN), 
      Units.inchesToMeters(MIN_HEIGHT_INCHES), 
      Units.inchesToMeters(MAX_HEIGHT_INCHES), 
      kSensorPhase, 
      0, VecBuilder.fill(0.1));
  }

  public static ClimberRight getInstance() {
    if (instance == null) {
      instance = new ClimberRight();
    }
    return instance;
  }
  public void setMotorBrake() {
    climberMotorRight.setIdleMode(IdleMode.kBrake);
  }

  public void setMotorCoast() {
    climberMotorRight.setIdleMode(IdleMode.kCoast);
  }

   //@Config()
  public void setSpeedVelocity(double velocity){
    m_pidController.setReference(ClimberLeft.inchesToTicks(velocity) * TIME_UNITS_OF_VELOCITY, ControlType.kVelocity, 0, kArbitraryFeedForward);
  }

  //@Config()
  public void setPosition(double in){
    m_pidController.setReference(ClimberLeft.inchesToTicks(in) * TIME_UNITS_OF_VELOCITY, ControlType.kSmartMotion, 0, kArbitraryFeedForward);
  }

  public void setPercentOutput(double speed){
    m_pidController.setReference(speed, ControlType.kVoltage,0, kArbitraryFeedForward); //functions the same as "SetVolt()" expect it isn't a set and forget method
  }

  public void setToZero(){
    m_pidController.setReference(0, ControlType.kSmartMotion, 0, kArbitraryFeedForward);
  }

  public void setInvertPosition(boolean invert){
    climberMotorRight.setInverted(invert);
  }

  public void setVolt(double volt) {
    climberMotorRight.setVoltage(volt);
  }

  //@Log(name = "placeholder", rowIndex = 0, columnIndex = 0)
  public double getCurrentSpeed(){
    return climberMotorRight.get();
  }

  //@Log(name = "placeholder", rowIndex = 0, columnIndex = 0)
  public double getspeedVelocity(){
    return (ClimberLeft.ticksToInches(climberMotorRight.get()) / TIME_UNITS_OF_VELOCITY);
  }

  //@Log(name = "placeholder", rowIndex = 0, columnIndex = 0)
  public double getPosition(){
    return ClimberLeft.ticksToInches(climberMotorRight.get());
  }

  public double getVoltage(){
    return climberMotorRight.getBusVoltage();
  }

  //@Log(name = "placeholder", rowIndex = 0, columnIndex = 0)
  public boolean getInvertPosition(){
    return climberMotorRight.getInverted();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
