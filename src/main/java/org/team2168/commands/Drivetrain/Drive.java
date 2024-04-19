// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class Drive extends Command {
  /** Creates a new Drive. */
  private Drivetrain drivetrain;
  private double forward;
  private double strafe;
  private double azimuth;

  public Drive(Drivetrain drivetrain, double forward, double strafe, double azimuth) {
    this.drivetrain = drivetrain;
    this.forward = forward;
    this.strafe = strafe;
    this.azimuth = azimuth;

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(forward, strafe, azimuth);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
