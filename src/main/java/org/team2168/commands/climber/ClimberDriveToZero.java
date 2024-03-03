// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDriveToZero extends Command {
  Climber climberLeft;
  Climber climberRight;
  double acceptableErrorTolerance = 0.5;
  /** Creates a new ClimberDriveToZero. */
  public ClimberDriveToZero(Climber climberLeft, Climber climberRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberLeft = climberLeft;
    this.climberRight = climberRight;

    addRequirements(climberLeft, climberRight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberLeft.setLeftToZero();
    climberRight.setRightToZero();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      climberLeft.setLeftVolt(0.0);
      climberLeft.setLeftToZero();
      climberRight.setRightVolt(0.0);
      climberRight.setRightToZero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (((0 - acceptableErrorTolerance) <= climberLeft.getLeftPositionInches()) && (climberLeft.getLeftPositionInches() <= 0)) ||
    ((climberLeft.getLeftPositionInches() <= acceptableErrorTolerance) && (0 <= climberLeft.getLeftPositionInches())) && 
    (((0 - acceptableErrorTolerance) <= climberRight.getRightPositionInches()) && (climberRight.getRightPositionInches() <= 0)) ||
    ((climberRight.getRightPositionInches() <= acceptableErrorTolerance) && (0 <= climberRight.getRightPositionInches()));
  
}
}
