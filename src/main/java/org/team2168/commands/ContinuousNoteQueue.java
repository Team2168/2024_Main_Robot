// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands;

import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;

import edu.wpi.first.wpilibj2.command.Command;

public class ContinuousNoteQueue extends Command {
  /** Creates a new ContinuousNoteQueue. */
  Indexer indexer;
  IntakeRoller intakeRoller;
  public ContinuousNoteQueue(Indexer indexer, IntakeRoller intakeRoller) {
    this.indexer = indexer;
    this.intakeRoller = intakeRoller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer, intakeRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!indexer.isNotePresent()) {
      indexer.setDriveIndexer(0.75);
      intakeRoller.setRollerSpeed(0.75);
    }
    else {
      indexer.setDriveIndexer(0.0);
      intakeRoller.setRollerSpeed(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setDriveIndexer(0.0);
    intakeRoller.setRollerSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
