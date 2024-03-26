// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.auto;

import org.team2168.commands.QueueNote;
import org.team2168.commands.Drivetrain.DriveWithLimelight;
import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
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
public class ThreeNoteAltSide extends SequentialCommandGroup {
  /** Creates a new ThreeNoteAltSide. */
  public ThreeNoteAltSide(Drivetrain drivetrain, IntakeRoller intakeRoller, IntakePivot intakePivot, Indexer indexer, Shooter shooter, ShooterPivot shooterPivot, Limelight limelight, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // drive back to the front of the speaker and score first note
      new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
      new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle),
      SwervePathUtil.getPathCommand("3_Note_Far_Alt_1", drivetrain, InitialPathState.DISCARDHEADING),
      new DriveWithLimelight(drivetrain, limelight, 2.5, true).withTimeout(1.0),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      // drive to and pick up second note
      new ParallelCommandGroup(
        SwervePathUtil.getPathCommand("3_Note_Far_Alt_2", drivetrain, InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -10.0).withTimeout(1.0),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // drive back to shooting position and shoot note
      new ParallelCommandGroup(
        SwervePathUtil.getPathCommand("3_Note_Far_Alt_3", drivetrain, InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
        new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1)
      ),
      new DriveWithLimelight(drivetrain, limelight, 2.5, true).withTimeout(1.5),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      // drive to and pick up third note
      new ParallelCommandGroup(
        SwervePathUtil.getPathCommand("3_Note_Far_Alt_2", drivetrain, InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -10.0).withTimeout(1.0),
        new QueueNote(intakeRoller, indexer, leds).withTimeout(4.0)
      ),
      // drive back to the shooting position and shoot note
      new ParallelCommandGroup(
        SwervePathUtil.getPathCommand("3_Note_Far_Alt_3", drivetrain, InitialPathState.PRESERVEODOMETRY),
        new SetIntakePivotPosition(intakePivot, -120.0).withTimeout(0.1),
        new SetIntakeSpeed(intakeRoller, 0.0).withTimeout(0.1)
      ),
      new DriveWithLimelight(drivetrain, limelight, 2.5, true).withTimeout(1.5),
      new DriveIndexeruntilnoNote(indexer, () -> 1.0).withTimeout(1.0),
      // stops shooter for end of auto
      new WaitCommand(1.0),
      new StopFlywheel(shooter)
    );
  }
}
