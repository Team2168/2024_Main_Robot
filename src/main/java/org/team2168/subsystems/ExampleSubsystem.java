// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private TalonFX _motor = new TalonFX(0);
 
  // The motor's inversion is such that moving clockwise is considered moving forward
  private InvertedValue inversion = InvertedValue.Clockwise_Positive;
  // The deadband for the motor--the minimum percentage output it needs to be commanded to go before actually moving
  // 0.05 is an arbitrary value
  private double neutralDeadband = 0.05; 
  // The maximum values the motor can be commanded to go, in percent
  // Values are arbitrary
  private double maxForwardOutput = 0.95;
  private double minForwardOutput = 0.0; //The motor is not allowed to move backwards 😱

  private double currentLimit = 20.0;
  private boolean currentLimitEnabled = true;
  private double currentThreshold = 25.0;
  private double currentTimeThreshold = 0.2;
  
  private double kP;
  private double kI;
  private double kD;

  private int feedbackSensor = 0;

  private static ExampleSubsystem instance = null;

  public static ExampleSubsystem getInstance() {
    if(instance == null)
      instance = new ExampleSubsystem();
    return instance;
  }

    /**
    * With CTRE's 6.0 API, setting up (configurating) motors has been revamped. 
    * <p> The new method involves configurations and the configurator. How this works is that
    * you load the configuration class with variables, then you pass the configurations 
    * to the configurator, which acutally pushes those variables onto the motor.
    *
    * <p> There are multiple ways to set the configurations of the motor, with two ways to 
    * set up the configuration and two ways to set member variables. This comment will 
    * go through the different methods:
    * 
    * <h3> Setting Up Configurations
    * 
    * <p> There are two types of configurations: the overarching TalonFXConfiguration and the
    * specific Configuration subclasses (MotorOutputConfigs, CurrentLimitsConfig, etc).
    * <p> The TalonFXConfiguration class holds all member variables that can be found
    * in each of the subclasses. 
    * <p> ex) var motorOneConfig = new TalonFXConfiguration();
    * 
    * <p> The subclasses hold specific member variables related to the subclass. For example,
    * the MotorOutputConfigs subclass would have variables related to the inversion of the 
    * motor, but not variables related to the motor playing music!
    * <p> ex) var motorOneMotorConfig = new MotorOutputConfigs();
    * 
    * <p> It does not really matter which way you decide to go when you configure your motors.
    * However, you may find a more organized system by creating and using the Configuration
    * subclasses
    * 
    * <h3> Setting Member Variables
    * <p> There are two ways to set member variables (which will then be applied to the motor you
    * are configurating): using the setter and accessing the member variable directly
    * <h4> Using the setter
    * <p> Each variable you can configure comes with a setter that doubles as a getter. These can 
    * be accessed by using a dot operator on the Configuration variable:
    * <p> ex) motorConfigs.withInversion(InvertedValue.Clockwise_Positive);
    * <h4> Accessing the member variable
    * <p> To change the member variable directly, you can use:
    * <p> ex) motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    * 
    * <p> While either method works, it is more conventional to use the setter!
    */
  private ExampleSubsystem() {
    _motor.getConfigurator().apply(new TalonFXConfiguration()); //sets the motor to its facotry default
    
    var motorConfigs = new MotorOutputConfigs();
    var currentConfigs = new CurrentLimitsConfigs();
    var gains = new Slot0Configs();
    var feedbackConfigs = new FeedbackConfigs();

    /* Motor Output Configurations */    
    motorConfigs.Inverted = inversion;
    motorConfigs.withNeutralMode(NeutralModeValue.Brake);
    motorConfigs.withDutyCycleNeutralDeadband(neutralDeadband);
    motorConfigs.withPeakForwardDutyCycle(maxForwardOutput);
    motorConfigs.withPeakReverseDutyCycle(minForwardOutput);
    
    /* Current Limits Configurations */
    currentConfigs.withSupplyCurrentLimit(currentLimit);
    currentConfigs.withSupplyCurrentLimitEnable(currentLimitEnabled);
    currentConfigs.withSupplyCurrentThreshold(currentThreshold);
    currentConfigs.withSupplyTimeThreshold(currentTimeThreshold);

    /* PID Gains Configurations */
    //setting gains with the dot operator
    gains.withKP(kP)
         .withKI(kI)
         .withKD(kD);
    //another way to set gains (accessing the member variable directly to change it):
    gains.kP = kP;
    gains.kI = kI;
    gains.kD = kD;

    /* Feedback Configurations */
    feedbackConfigs.withFeedbackRemoteSensorID(feedbackSensor); //normally, the parameter should be calling the sensor ID from Constants
                                                                //ex) CanDevices.SensorOne  
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
