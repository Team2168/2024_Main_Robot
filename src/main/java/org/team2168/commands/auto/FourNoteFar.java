// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.QueueNote;
import org.team2168.commands.Drivetrain.DriveWithLimelight;
import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
import org.team2168.commands.ShooterCommands.ShootAndControlHoodFromDistance;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.StopFlywheel;
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
import org.team2168.subsystems.ShooterSubsystem.Shooter.SHOOTING_RPS;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot.SHOOTING_ANGLE;
import org.team2168.utils.SwervePathUtil;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourNoteFar extends SequentialCommandGroup {
  /** Creates a new FourNoteFar. */
  public FourNoteFar(Drivetrain drivetrain, IntakeRoller intakeRoller, IntakePivot intakePivot, Indexer indexer, Shooter shooter, ShooterPivot shooterPivot, Limelight limelight, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // shoot first note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new ControlShooterAndHood(shooter, shooterPivot, SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
      new WaitCommand(0.6),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(0.5),
      // drive to and pick up second note
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new StopFlywheel(shooter)
        ),
        new FollowPathPlannerPath(drivetrain, "4_Note_Far_1", InitialPathState.DISCARDHEADING),
        // new FollowInitialPath(drivetrain, "4_Note_Far_1"),
        new SetIntakePivotPosition(intakePivot, -12.5).withTimeout(1.0),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(2.7)
      ),
      // aim and shoot second note from this position
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new ParallelCommandGroup(
        new DriveWithLimelight(drivetrain, limelight, 1.0, true),
        new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight)
      ).withTimeout(0.75),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(0.25),
      // drive to and pick up third note
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new StopFlywheel(shooter)
        ),
        new FollowPathPlannerPath(drivetrain, "4_Note_Far_2", InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -12.5).withTimeout(1.0),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // drive to scoring position and score 3rd note
      new ParallelCommandGroup(
        new FollowPathPlannerPath(drivetrain, "4_Note_Far_3", InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(1.0),
        new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1)
      ),
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new ParallelCommandGroup(
      new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight),
      new DriveWithLimelight(drivetrain, limelight, 1.0, true)
      ).withTimeout(1.5),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(0.27),
      // drive to and pick up fourth note
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new StopFlywheel(shooter)
        ),
        new FollowPathPlannerPath(drivetrain, "4_Note_Far_4", InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -12.5).withTimeout(1.0),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // drive to scoring position and score 4th note
      new ParallelCommandGroup(
        new FollowPathPlannerPath(drivetrain, "4_Note_Far_5", InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(1.0),
        new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1)
      ),
      new ParallelCommandGroup(
        new DriveWithLimelight(drivetrain, limelight, 1.0, true),
        new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight)
      ).withTimeout(1.2),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      new WaitCommand(0.25),
      new StopFlywheel(shooter)
    );
  }
}
