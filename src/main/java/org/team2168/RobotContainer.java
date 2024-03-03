// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import javax.sql.ConnectionPoolDataSource;

import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.Autos;
import org.team2168.commands.ContinuousNoteQueue;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.QueueNote;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.indexer.DriveIndexeruntilnoNote;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.Indexer;
//import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.IntakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  static RobotContainer instance = null;

  private final IntakeRoller intakeRoller = IntakeRoller.getInstance();
  private final IntakePivot intakePivot = IntakePivot.getInstance();
  private final Indexer indexer = Indexer.getInstance();

  OI oi = OI.getInstance();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final Indexer indexer = Indexer.getInstance();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    Logger.configureLoggingAndConfig(this, false);
  }

 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // oi.testJoystick.ButtonX().whileTrue(new SetIntakeSpeed(intakeRoller, .5));
    // oi.testJoystick.ButtonY().whileTrue(new SetIntakeSpeed(intakeRoller, .4));
    // oi.operatorJoystick.ButtonLeftBumper().whileTrue(new ContinuousNoteQueue(indexer, intakeRoller));
    oi.operatorJoystick.ButtonLeftBumper().whileTrue(new RepeatCommand(new QueueNote(intakeRoller, indexer))); // TODO: test
    oi.operatorJoystick.ButtonLeftBumper().whileTrue(new SetIntakePivotPosition(intakePivot, 0.0)).onFalse(new SetIntakePivotPosition(intakePivot, -120.0));
    oi.operatorJoystick.ButtonRightBumper().whileTrue(new DriveIndexeruntilnoNote(indexer, () -> 1.0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
}
