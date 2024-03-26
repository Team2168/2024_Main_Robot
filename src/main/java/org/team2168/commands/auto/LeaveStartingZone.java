// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.Drivetrain.Drive;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.IntakePivot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeaveStartingZone extends SequentialCommandGroup {
  /** Creates a new LeaveStartingZone. */
  public LeaveStartingZone(Drivetrain drivetrain, IntakePivot intakePivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetIntakePivotPosition(intakePivot, -120.0).raceWith(
      new Drive(drivetrain, -0.25, 0, 0).withTimeout(2.5)), //TODO: verify that this is quick and enough time to back out
      new Drive(drivetrain, 0, 0, 0)
    );
  }
}
