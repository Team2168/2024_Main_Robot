// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.subsystems.Drivetrain;
import org.team2168.utils.SwervePathUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathFindToAmp extends SequentialCommandGroup {
  /** Creates a new PathFindToAmp. */
  public PathFindToAmp(Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      SwervePathUtil.pathFindToAmp(drive)
    );
  }
}
