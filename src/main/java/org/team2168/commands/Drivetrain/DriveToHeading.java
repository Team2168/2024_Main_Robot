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
  double ccwHeading;
  int numLoops;
  final double ERROR_TOLERANCE = 3.0; // in degrees
  final int ACCEPTED_LOOPS = 10;

  double kP = 0.01;
  double kI = 0.0;
  double kD = 0.00025;

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
    ccwHeading = -drivetrain.getHeading();

    if (ccwHeading < 0.0) {
      ccwHeading = (ccwHeading % 360.0) + 360.0; // converts ccw heading to lowest positive possible value
    }
    else {
      ccwHeading = ccwHeading % 360; // converts ccw to lowest possible positive value
    }

    if (Math.abs(ccwHeading % 360.0 - angle) < 180.0) {
    drivetrain.drive(0.0, 0.0, drivePID.calculate((angle - ((ccwHeading) % 360.0)))); // check math
    // drivetrain.driveWithKinematics(0.0, 0.0, drivePID.calculate(angle - drivetrain.getHeading() % 360.0));
    }
    else {
      drivetrain.drive(0.0, 0.0, drivePID.calculate((ccwHeading % 360.0 - angle))); // check math
      // drivetrain.driveWithKinematics(0.0, 0.0, drivePID.calculate(drivetrain.getHeading() % 360.0 - angle));
    }

    if (Math.abs(ccwHeading - angle) < ERROR_TOLERANCE) {
      numLoops++;
    }
    else {
      numLoops = 0;
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
    return (numLoops >= ACCEPTED_LOOPS);
  }
}
