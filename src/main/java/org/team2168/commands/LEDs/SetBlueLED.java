// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
/**
 * @param
 * Sets the blue LED on or off
 */
public class SetBlueLED extends Command {
  /** Creates a new SetBlueLED. */
  private LEDs leds;
  private boolean isOn;
  
  public SetBlueLED(LEDs leds, boolean isOn) {
    /** 
     * 
     * Gives the sub system some things that it depends on
     */
  
    this.leds = leds;
    this.isOn = isOn;

    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.bluelight(true);
leds.redlight(false);
leds.greenlight(false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.bluelight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
