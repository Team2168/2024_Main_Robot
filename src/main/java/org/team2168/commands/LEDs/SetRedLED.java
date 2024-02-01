// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.LEDs;

import org.team2168.subsystems.LEDs;
 
import edu.wpi.first.wpilibj2.command.Command;

public class SetRedLED extends Command {
  /** Creates a new SetRedLED. */
  private LEDs leds;
  private boolean Ison;
  public SetRedLED(LEDs leds, boolean Ison) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    this.Ison = Ison;

    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //leds.red(Ison);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
