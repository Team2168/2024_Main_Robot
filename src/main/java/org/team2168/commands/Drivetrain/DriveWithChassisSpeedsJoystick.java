// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Drivetrain;

import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithChassisSpeedsJoystick extends Command {
  /** Creates a new DriveWithChassisSpeedsJoystick. */
  Drivetrain drive;
  OI oi;
  double chassisRot;
  private double kDriveInvert = 1.0;
  private SlewRateLimiter rotationRateLimiter;

  public DriveWithChassisSpeedsJoystick(Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = OI.getInstance();
    rotationRateLimiter = new SlewRateLimiter(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.joystickChooser.getSelected().equals("flight")) {
      if (oi.driverJoystick.isPressedButtonA()) {
        chassisRot = 0.4;
      }
      else if (oi.driverJoystick.isPressedButtonB()) {
        chassisRot = -0.4;
      }
      else {
        chassisRot = 0.0;
      }
    }
    else {
      chassisRot = oi.getDriverJoystickZValue();
    }

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      kDriveInvert = -1.0;
    }

    drive.driveWithKinematics(oi.getLimitedDriverJoystickYValue() * kDriveInvert, oi.getLimitedDriverJoystickXValue() * kDriveInvert, chassisRot);
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
