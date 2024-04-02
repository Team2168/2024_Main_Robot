// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

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
        new QueueNote(intakeRoller, indexer, leds).withTimeout(1.5)
      ),
      // drives towards 3rd note and shoots 2nd note while moving
      new ParallelCommandGroup(
        new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
        new SequentialCommandGroup(
          new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
          new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
            // index for short portion of time
            new DriveIndexeruntilNote(indexer, () -> 0.75).withTimeout(0.75),
            new WaitCommand(0.85),
          new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(0.5),
          new SetIntakePivotPosition(intakePivot, -10.0).withTimeout(0.5),
          new StopFlywheel(shooter),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(1.0)
        ),
        new FollowPathPlannerPath(drivetrain, "4_Note_Close_2", InitialPathState.PRESERVEODOMETRY)
      ),
      // drives towards 4th note and shoots 3rd note while moving
      new ParallelCommandGroup(
        new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
        new SequentialCommandGroup(
          new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
          new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
          // index for short portion of time
            new DriveIndexeruntilNote(indexer, () -> 0.75).withTimeout(1.0),
            new WaitCommand(0.75),
          new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(0.5),
          new SetIntakePivotPosition(intakePivot, -10.0).withTimeout(0.5),
          new StopFlywheel(shooter),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(1.5)
        ),
        new FollowPathPlannerPath(drivetrain, "4_Note_Close_3", InitialPathState.PRESERVEODOMETRY)
      ),
      // drives to speaker and shoots 4th note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new FollowPathPlannerPath(drivetrain, "4_Note_Close_4", InitialPathState.PRESERVEODOMETRY),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0),
      new WaitCommand(0.5),
      new StopFlywheel(shooter)
      );
  }
}
