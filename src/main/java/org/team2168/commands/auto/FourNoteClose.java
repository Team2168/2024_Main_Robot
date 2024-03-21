// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.QueueNote;
import org.team2168.commands.Drivetrain.DriveWithLimelight;
import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
import org.team2168.commands.indexer.DriveIndexeruntilnoNote;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakePivot;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;
import org.team2168.utils.SwervePathUtil;
import org.team2168.utils.SwervePathUtil.InitialPathState;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourNoteClose extends SequentialCommandGroup {
  /** Creates a new FourNoteClose. */
  public FourNoteClose(Drivetrain drivetrain, IntakeRoller intakeRoller, IntakePivot intakePivot, Indexer indexer, Shooter shooter, ShooterPivot shooterPivot, Limelight limelight, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // fires first note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.5),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
      new WaitCommand(1.0),
      new DriveIndexeruntilnoNote(indexer, () -> 0.75).withTimeout(1.0),
      // drives to and picks up second
      new ParallelCommandGroup(
        SwervePathUtil.getPathCommand("4_Note_Close_1", drivetrain, InitialPathState.DISCARDHEADING),
        new SetIntakePivotPosition(intakePivot, -7.5).withTimeout(0.5),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // shoots second note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.STARTING_ZONE_LINE.shooterRPS, ShooterPivot.SHOOTING_ANGLE.STARTING_ZONE_LINE.shooterAngle),
      new DriveWithLimelight(drivetrain, limelight, 1.0, true).withTimeout(1.0),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      // drives to and picks up 3rd note
      new ParallelCommandGroup(
        SwervePathUtil.getPathCommand("4_Note_Close_2", drivetrain, InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -7.5).withTimeout(0.5),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // shoots 3rd note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new DriveWithLimelight(drivetrain, limelight, 1.0, true).withTimeout(1.0),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      // druves to and picks up 4th note
      new ParallelCommandGroup(
        SwervePathUtil.getPathCommand("4_Note_Close_3", drivetrain, InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -7.5).withTimeout(0.5),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // shoots 4th note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new DriveWithLimelight(drivetrain, limelight, 1.0, true).withTimeout(1.0),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0)
    );
  }
}
