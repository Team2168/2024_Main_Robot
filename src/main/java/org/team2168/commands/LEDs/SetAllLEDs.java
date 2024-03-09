// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
//a way to set every led to something at once
public class SetAllLEDs extends Command {
  /** Creates a new SetAllLEDs. */
  private LEDs leds;
  private boolean redOn;
  private boolean blueOn;
  private boolean greenOn;
  public SetAllLEDs(LEDs leds, boolean redOn, boolean greenOn, boolean blueOn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    this.redOn = redOn;
    this.blueOn = blueOn;
    this.greenOn = greenOn;

    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      leds.redlight(redOn);
    leds.greenlight(greenOn);
    leds.bluelight(blueOn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
