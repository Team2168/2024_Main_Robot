// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SetHeading extends Command {
  /** Creates a new SetHeading. */
  Drivetrain drivetrain;
  double heading;
  public SetHeading(Drivetrain drivetrain, double heading) {
    this.drivetrain = drivetrain;
    this.heading = heading;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setHeading(heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getHeading() - heading) < 0.5;
  }
}
