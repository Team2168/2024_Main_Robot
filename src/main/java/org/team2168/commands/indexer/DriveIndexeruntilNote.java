// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.indexer;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveIndexeruntilNote extends Command {
  private Indexer indexer;
  private DoubleSupplier speed;
 /** it drives the indexer until there is a note
   * @param indexer indexer subsystem to be used for method
   * @param speed to set the indexer at
   */
  public DriveIndexeruntilNote(Indexer indexer, DoubleSupplier speed) {
    this.indexer = indexer;
    this.speed = speed;
    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.setDriveIndexer(speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.setDriveIndexer(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.isNotePresent();
  }
}
