// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.intakerRoller;

import org.team2168.subsystems.IntakeRoller;

import edu.wpi.first.wpilibj2.command.Command;

public class SetIntakeSpeed extends Command {
  private IntakeRoller iRoller;
  private double speed;

  /**
   * sets the speed of the intake
   * @param iRoller the intake instance
   * @param speed speed of the intake roller from -1 to 1
   */
  public SetIntakeSpeed(IntakeRoller iRoller, double speed) {
    this.iRoller = iRoller;
    this.speed = speed;
    addRequirements(iRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    iRoller.setRollerSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    iRoller.setRollerSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
