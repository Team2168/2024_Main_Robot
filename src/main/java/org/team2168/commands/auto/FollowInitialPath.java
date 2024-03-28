// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowInitialPath extends SequentialCommandGroup {
  /** Creates a new FollowInitialPath.
   * 
   * Intended to be used at the start of auto to reset pose and heading for the rest of the match
   */
  public FollowInitialPath(Drivetrain drivetrain, String pathName) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(drivetrain.followInitialPath(pathName));
  }
}
