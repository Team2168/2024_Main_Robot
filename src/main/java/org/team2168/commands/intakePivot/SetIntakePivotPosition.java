// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.intakePivot;

import org.team2168.subsystems.IntakePivot;

import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakePivotPosition extends Command {
  private IntakePivot iPivot;
  private double position;

  /**
   * sets the positon of the intake in degrees
   * @param iPivot intake positon instance
   * @param position the position of intake
   */
  public SetIntakePivotPosition(IntakePivot iPivot, double position) {
    this.iPivot = iPivot;
    this.position = position;
    addRequirements(iPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    iPivot.setIntakePivotPosition(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
