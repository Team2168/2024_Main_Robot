// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.ShooterCommands;

import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;

import edu.wpi.first.wpilibj2.command.Command;

public class ControlShooterAndHood extends Command {
  private Shooter shooter;
  private ShooterPivot shooterPivot;
  private double velocity;
  private double angle;
  public ControlShooterAndHood(Shooter shooter, ShooterPivot shooterPivot, double velocity, double angle) {
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.velocity = velocity;
    this.angle = angle;
    addRequirements(shooter, shooterPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(velocity);
    shooterPivot.setPositionDegrees(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
