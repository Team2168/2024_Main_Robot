// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.QueueNote;
import org.team2168.commands.Drivetrain.DriveWithLimelight;
import org.team2168.commands.ShooterCommands.ShootAndControlHoodFromDistance;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.StopFlywheel;
import org.team2168.commands.indexer.DriveIndexeruntilnoNote;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Drivetrain.InitialPathState;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakePivot;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.LEDs;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WPIThreeNote extends SequentialCommandGroup {
  /** Creates a new WPIThreeNote. */
  public WPIThreeNote(Drivetrain drivetrain, IntakeRoller intakeRoller, IntakePivot intakePivot, Indexer indexer, Shooter shooter, ShooterPivot shooterPivot, Limelight limelight, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // shoot first note
      new ParallelCommandGroup(
      new DriveWithLimelight(drivetrain, limelight, 1.0, true),
      new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight)
      ).withTimeout(2.0),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      //drive back to second note, pick up and shoot
      new ParallelCommandGroup(
        new FollowPathPlannerPath(drivetrain, "WPI3_Note_1", InitialPathState.DISCARDHEADING),
        new SetIntakePivotPosition(intakePivot, -12.5).withTimeout(1.0),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(2.5)
      ),
      // shoot
      new ParallelCommandGroup(
      new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight),
      new DriveWithLimelight(drivetrain, limelight, 1.0, true)
      ).withTimeout(1.2),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      // drive to and pick up third note
      new ParallelCommandGroup(
        new FollowPathPlannerPath(drivetrain, "WPI3_Note_2", InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -12.5).withTimeout(1.0),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // drive back and shoot third note
      new ParallelCommandGroup(
        new FollowPathPlannerPath(drivetrain, "WPI3_Note_3", InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
        new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1)
      ),
      new ParallelCommandGroup(
      new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight),
      new DriveWithLimelight(drivetrain, limelight, 1.0, true)
      ).withTimeout(1.2),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      // stops shooter for end of auto
      new WaitCommand(1.0),
      new StopFlywheel(shooter)

    );
  }
}
