// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.QueueNote;
import org.team2168.commands.Drivetrain.DriveWithLimelight;
import org.team2168.commands.Drivetrain.SetHeading;
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
public class TwoNoteAuto extends SequentialCommandGroup {
  /** Creates a new TwoNoteAuto. */
  Drivetrain drivetrain;
  IntakeRoller intakeRoller;
  IntakePivot intakePivot;
  Shooter shooter;
  ShooterPivot shooterPivot;
  Limelight limelight;
  LEDs leds;
  public TwoNoteAuto(Drivetrain drivetrain, IntakeRoller intakeRoller, IntakePivot intakePivot, Indexer indexer, Shooter shooter, ShooterPivot shooterPivot, Limelight limelight, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.drivetrain = drivetrain;
    this.intakeRoller = intakeRoller;
    this.intakePivot = intakePivot;
    this.shooter = shooter;
    this.shooterPivot = shooterPivot;
    this.limelight = limelight;
    this.leds = leds;
    addCommands( // shoots first note
      // new SetHeading(drivetrain, 180.0),
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.5),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle).withTimeout(1.0),
      new WaitCommand(1.0),
      new DriveIndexeruntilnoNote(indexer, () -> 0.75).withTimeout(1.0),
      // moves back to pick up second note
      new FollowPathPlannerPath(drivetrain, "Move_Back_Speaker", InitialPathState.DISCARDHEADING).raceWith(
        // new FollowInitialPath(drivetrain, "Move_Back_Speaker").raceWith(
        new SetIntakePivotPosition(intakePivot, -15.0),
        new QueueNote(intakeRoller, indexer, leds)
      ),
      new QueueNote(intakeRoller, indexer, leds).withTimeout(2.0),
      // drives back upon intaking, stows intake
      // new FollowPathPlannerPath(drivetrain, "Move_To_Speaker", InitialPathState.PRESERVEODOMETRY).raceWith(
      //   new SetIntakePivotPosition(intakePivot, -120.0),
      //   new SetIntakeSpeed(intakeRoller, 0.0)
      // ),
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1),
      new DriveWithLimelight(drivetrain, limelight, 1.5, true).withTimeout(1.5),
      new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight).withTimeout(3.0),
      // shoots second note
      new DriveIndexeruntilnoNote(indexer, () -> 0.75).withTimeout(1.0),
      new WaitCommand(0.75),
      new StopFlywheel(shooter)
    );
  }
}
