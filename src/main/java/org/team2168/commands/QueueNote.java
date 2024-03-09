// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.commands.indexer.DriveIndexeruntilNote;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class QueueNote extends SequentialCommandGroup {
  /** Creates a new QueueNote. */
  public QueueNote(IntakeRoller iRoller, Indexer indexer, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetIntakeSpeed(iRoller, 0.6).raceWith(new DriveIndexeruntilNote(indexer, () -> 0.75, leds)
      )
    );
  }
}
