// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDriveToZero extends Command {
  Climber climber;
  double acceptableErrorTolerance = 0.5;
  /** Creates a new ClimberDriveToZero. */
  public ClimberDriveToZero(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setToZero();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      climber.setVolt(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((0 - acceptableErrorTolerance) <= climber.getPositionInches()) && (climber.getPositionInches() <= 0)) ||
    ((climber.getPositionInches() <= acceptableErrorTolerance) && (0 <= climber.getPositionInches())); 
}
}
