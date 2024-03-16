// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import org.team2168.Constants.CANDevices;

import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Log;


public class IntakeRoller extends SubsystemBase {
  private static CANSparkMax intakeRollerOne = new CANSparkMax(CANDevices.intakeRoller, CANSparkLowLevel.MotorType.kBrushless);

  private static IntakeRoller instance = null;
  
  private static RelativeEncoder intakeRollerEncoder = intakeRollerOne.getEncoder();

  public static IntakeRoller getInstance() {
    if(instance == null)
    instance = new IntakeRoller();
    return instance;
  }
  
  private final double minuteInHundredMs = 600.0;
  private final double TICKS_PER_REV = 2048;
  private final double GEAR_RATIO = 4.628;
  private final int SMART_CURRENT_LIMIT = 15;
  private boolean isInverted = false;
  private IdleMode coast = IdleMode.kCoast;


  private IntakeRoller() {

    intakeRollerOne.restoreFactoryDefaults();

    intakeRollerOne.setInverted(isInverted);
    intakeRollerOne.setIdleMode(coast);
    intakeRollerOne.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    
  }
  
  
  /**
   * sets the speed in percentage
   * @param speed value is between -1.0 and 1.0
   */
  public void setRollerSpeed(double speed) {
    intakeRollerOne.set(speed);
  }

  /**
   * converts RPM to ticks per one hundred ms
   * @param speedRPM amount of speed in rpm
   * @return amount of ticks from rpm
   */

  private double RPMToTicksPerOneHundredMS(double speedRPM) {
    return (speedRPM/minuteInHundredMs) * (TICKS_PER_REV/GEAR_RATIO);
  }
  
  /**
   * converts ticks per one hundred ms to rpm
   * @param ticksPerHundredMs amount of ticks per hundred ms
   * @return amount of rpm from ticks
   */
  private double TicksPerOneHundredMSToRPM(double ticksPerHundredMs) {
    return ticksPerHundredMs * (GEAR_RATIO/TICKS_PER_REV) * minuteInHundredMs;
  }

@Log(name = "speed (rotations per minutes)", rowIndex = 3, columnIndex = 1)

  /**
   * gets the speed in rpm
   * @return the speedrpm
   */
  public double getSpeedRPM () {
    return TicksPerOneHundredMSToRPM(intakeRollerEncoder.getVelocity());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
