// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.ClimberLeft;
import org.team2168.subsystems.ClimberRight;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberDriveToZero extends Command {
  ClimberLeft climberLeft;
  ClimberRight climberRight;
  /** Creates a new ClimberDriveToZero. */
  public ClimberDriveToZero(ClimberLeft climberLeft, ClimberRight climberRight) {
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
    climberLeft.setToZero();
    climberRight.setToZero();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      climberLeft.setVolt(0.0);
      climberLeft.setToZero();
      climberRight.setVolt(0.0);
      climberRight.setToZero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climberLeft.getPositionInches() == 0) && (climberRight.getPositionInches() == 0);
  
}
}
