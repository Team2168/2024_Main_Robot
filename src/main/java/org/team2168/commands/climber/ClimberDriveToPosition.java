// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.ClimberLeft;
import org.team2168.subsystems.ClimberRight;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDriveToPosition extends Command {
  ClimberLeft climberLeft;
  ClimberRight climberRight;
  double inches;
  double acceptableErrorTolerance = 0.5;
  /** Creates a new ClimberDriveToPosition. */
  public ClimberDriveToPosition(ClimberLeft climberLeft, ClimberRight climberRight, double in) {
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
    climberLeft.setPosition(inches);
    climberRight.setPosition(inches);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((climberLeft.getPosition() >= (inches - acceptableErrorTolerance) && climberLeft.getPosition() <= (inches + acceptableErrorTolerance)) && 
    (climberRight.getPosition() >= (inches - acceptableErrorTolerance) && climberRight.getPosition() <= (inches + acceptableErrorTolerance)));
  }
  
}
