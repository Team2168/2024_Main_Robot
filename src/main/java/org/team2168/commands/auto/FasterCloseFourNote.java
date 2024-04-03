// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.ContinuousNoteQueue;
import org.team2168.commands.QueueNote;
import org.team2168.commands.Drivetrain.DriveWithLimelight;
import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
import org.team2168.commands.ShooterCommands.ShootAndControlHoodFromDistance;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.StopFlywheel;
import org.team2168.commands.indexer.DriveIndexeruntilNote;
import org.team2168.commands.indexer.DriveIndexeruntilnoNote;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakePivot;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Drivetrain.InitialPathState;
import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FasterCloseFourNote extends SequentialCommandGroup {
  /** Creates a new FasterCloseFourNote. */
  public FasterCloseFourNote(Drivetrain drivetrain, IntakeRoller intakeRoller, IntakePivot intakePivot, Indexer indexer, Shooter shooter, ShooterPivot shooterPivot, Limelight limelight, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // fires first note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
      new WaitCommand(0.6),
      new DriveIndexeruntilnoNote(indexer, () -> 0.75).withTimeout(0.25),
      // drives to and picks up second
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new StopFlywheel(shooter)
        ),
        new FollowPathPlannerPath(drivetrain, "4_Note_Close_1", InitialPathState.DISCARDHEADING),
        // new FollowInitialPath(drivetrain, "4_Note_Close_1"),
        new SetIntakePivotPosition(intakePivot, -10.0).withTimeout(0.5),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(2.8)
      ),
      // shoots second note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new PathFindToSpeaker(drivetrain),
      // new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.STARTING_ZONE_LINE.shooterRPS, ShooterPivot.SHOOTING_ANGLE.STARTING_ZONE_LINE.shooterAngle),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
      new WaitCommand(0.25),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(0.25),
      new WaitCommand(0.2),
      new StopFlywheel(shooter),
      // drives to and picks up 3rd note
      new ParallelCommandGroup(
        new FollowPathPlannerPath(drivetrain, "4_Note_Subwoofer_2", InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -10.0).withTimeout(0.5),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // shoots 3rd note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new PathFindToSpeaker(drivetrain),
      // new DriveWithLimelight(drivetrain, limelight, 1.0, true).withTimeout(1.0),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
      new WaitCommand(0.2),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(0.25),
      new WaitCommand(0.2),
      new StopFlywheel(shooter),
      // druves to and picks up 4th note
      new ParallelCommandGroup(
        new FollowPathPlannerPath(drivetrain, "4_Note_Subwoofer_3", InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -10.0).withTimeout(0.5),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(3.3)
      ),
      // shoots 4th note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new PathFindToSpeaker(drivetrain),
      // new DriveWithLimelight(drivetrain, limelight, 1.0, true).withTimeout(1.0),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
      new WaitCommand(0.5),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      new WaitCommand(0.5),
      new StopFlywheel(shooter)
      );
  }
}
