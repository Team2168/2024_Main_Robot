// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Drivetrain;

import org.team2168.commands.auto.PathFindToAmp;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignWithAmp extends SequentialCommandGroup {
  /** Creates a new AlignWithAmp. */

  public AlignWithAmp(Drivetrain drivetrain, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      drivetrain.pathFindToAmp(),
      new DriveToHeading(drivetrain, 90.0).withTimeout(1.5),
      new StrafeToTagPosition(drivetrain, limelight, 1.5).withTimeout(2.0),
      new Drive(drivetrain, 0.0, -0.25, 0.0).withTimeout(2.5)
    );
  }
}
