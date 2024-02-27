// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.ShooterCommands.ShooterFlywheel;

import org.team2168.subsystems.ShooterSubsystem.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterVelocity extends Command {
  private Shooter shooter;
  private double velocity;
  
  public SetShooterVelocity(Shooter shooter, double velocity) {
    this.shooter = shooter;
    this.velocity = velocity;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(velocity);
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
