// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
import org.team2168.commands.indexer.DriveIndexeruntilnoNote;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;
import org.team2168.utils.SwervePathUtil;
import org.team2168.utils.SwervePathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteAuto extends SequentialCommandGroup {
  /** Creates a new OneNoteAuto. */
  Drivetrain drivetrain;
  Shooter shooter;
  ShooterPivot shooterPivot;
  Limelight limelight;
  public OneNoteAuto(Drivetrain drivetrain, Indexer indexer, Shooter shooter, ShooterPivot shooterPivot, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.limelight = limelight;
    addCommands(
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle).withTimeout(1.0),
      new DriveIndexeruntilnoNote(indexer, () -> 0.75),
      SwervePathUtil.getPathCommand("Move_Back_Speaker", drivetrain, InitialPathState.DISCARDHEADING)
    );
  }
}
