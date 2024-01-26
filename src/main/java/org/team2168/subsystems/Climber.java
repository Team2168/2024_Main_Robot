// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team2168.Constants;
import org.team2168.Constants.ClimberMotors;
import org.team2168.utils.TalonFXHelper;

public class Climber extends SubsystemBase {

  private TalonFXHelper climberMotor;

  static Climber instance = null;
  private static final double TIME_UNITS_OF_VELOCITY = 0.1;
  private static final double TICKS_PER_REV = 2048;
  private static final double GEAR_RATIO = 0; //placeholder number
  private static final double SPROCKET_RADIUS_IN = 0; //placeholder number
  private static final double INCHES_PER_REV = SPROCKET_RADIUS_IN * 2 * Math.PI;

  private static final double kP = 0;//placeholder
  private static final double kI = 0;//placeholder
  private static final double kD = 0;//placeholder
  private static final double kF = 0;//placeholder
  private static final double kArbitraryFeedForward = 0;//placeholder

  private static final double kPeakOutput = 1.0;
  private static final double NEUTRAL_DEADBAND = 0.001;
  private static final double ACCELERATION_LIMIT = inchesToTicks(21.68 * 3.0) * TIME_UNITS_OF_VELOCITY; //placeholder
  private static final double CRUISE_VELOCITY_LIMIT = inchesToTicks(21.68 * 2.5) * TIME_UNITS_OF_VELOCITY; //placeholder

  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30; //placeholder
  private static boolean kSensorPhase = false;

  private static final double CURRENT_LIMIT = 0.0; //it limits when the feature is activited (in amps)
  private static final double THRESHOLD_CURRENT = 0.0; //it tells what the threshold should be for the limit to be activited (in amps)
  private static final double THRESHOLD_TIME = 0.0; //time in seconds of when the limiting should happen after the threshold has been overreached

  /** Creates a new Climber. */
  public Climber() {
      climberMotor = new TalonFXHelper(ClimberMotors.CLIMBER_MOTOR);


  }

  public static Climber getInstance(){
    if (instance == null){
      instance = new Climber();
    }
    return instance;
  }

  public static double degreesToTicks(double degrees){
    return (degrees / 360 * TICKS_PER_REV);
  }

  public static double ticksToDegrees(double ticks){
    return (ticks / TICKS_PER_REV * 360);
  }

  private static double inchesToTicks(double inches) {
    return (inches / INCHES_PER_REV) * GEAR_RATIO * TICKS_PER_REV;
  }

  public static double ticksToInches(double ticks){
    return ((ticks / TICKS_PER_REV) /GEAR_RATIO) * INCHES_PER_REV;
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
