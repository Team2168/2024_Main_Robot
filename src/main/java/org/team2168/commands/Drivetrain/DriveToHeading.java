// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Drivetrain;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToHeading extends Command {
  /** Creates a new DriveToHeading. */
  Drivetrain drivetrain;
  double angle;
  final double ERROR_TOLERANCE = 0.5; // in degrees

  double kP = 0.3;
  double kI = 0.0;
  double kD = 0.0;

  PIDController drivePID = new PIDController(kP, kI, kD);

  public DriveToHeading(Drivetrain drivetrain, double angle) {
    this.drivetrain = drivetrain;
    this.angle = angle; // in degrees

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(drivetrain.getHeading() - angle) % 360.0 < 180.0) {
    drivetrain.drive(0.0, 0.0, drivePID.calculate((angle - drivetrain.getHeading() % 360.0)));
    }
    else {
      drivetrain.drive(0.0, 0.0, drivePID.calculate((drivetrain.getHeading() % 360.0 - angle)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(drivetrain.getHeading()) < ERROR_TOLERANCE);
  }
}
