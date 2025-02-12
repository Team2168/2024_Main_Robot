// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.ShooterCommands;

import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;

import edu.wpi.first.wpilibj2.command.Command;

public class ShootAndControlHoodFromDistance extends Command {
  private Shooter shooter;
  private ShooterPivot shooterPivot;
  private Limelight limelight;
  private double distanceMeters = 1.57; //need limelight code for this
  public ShootAndControlHoodFromDistance(Shooter shooter, ShooterPivot shooterPivot, Limelight limelight) {
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.limelight = limelight;
    addRequirements(shooter, shooterPivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceMeters = limelight.calculateDistance();
    shooter.setMotorSpeedFromDistance(distanceMeters);
    shooterPivot.setShooterAngleFromDistance(distanceMeters);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
