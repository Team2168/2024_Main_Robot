// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.ShooterCommands.ShooterPivot;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveShooterHoodWithController extends Command {
  private DoubleSupplier buttonOutput;
  private ShooterPivot shooterPivot;

  public MoveShooterHoodWithController(ShooterPivot shooterPivot, DoubleSupplier buttonOutput) {
    this.shooterPivot = shooterPivot;
    this.buttonOutput = buttonOutput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterPivot.setPositionDegrees(shooterPivot.getAngle() + buttonOutput.getAsDouble());
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
