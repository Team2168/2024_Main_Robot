// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.ShooterCommands.ShooterPivot;

import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;

import edu.wpi.first.wpilibj2.command.Command;

public class SetAngle extends Command {
  private ShooterPivot shooterPivot;
  private double shooterAngle;
  public SetAngle(ShooterPivot shooterPivot, double shooterAngle) {
    this.shooterPivot = shooterPivot;
    this.shooterAngle = shooterAngle;
    addRequirements(shooterPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterPivot.setPositionDegrees(shooterAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterPivot.setPercentOutput(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
