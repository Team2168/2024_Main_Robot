// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDriveToPosition extends Command {
  Climber climberLeft;
  Climber climberRight;
  double inches;
  double acceptableErrorTolerance = 0.5;
  /** Creates a new ClimberDriveToPosition. */
  public ClimberDriveToPosition(Climber climberLeft, Climber climberRight, double in) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberLeft = climberLeft;
    this.climberRight = climberRight;
    inches = in;

    addRequirements(climberLeft, climberRight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberLeft.setLeftPosition(inches);
    climberRight.setRightPosition(inches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((climberLeft.getLeftPositionInches() >= (inches - acceptableErrorTolerance) && climberLeft.getLeftPositionInches() <= (inches + acceptableErrorTolerance)) && 
    (climberRight.getRightPositionInches() >= (inches - acceptableErrorTolerance) && climberRight.getRightPositionInches() <= (inches + acceptableErrorTolerance)));
  }
  
}
