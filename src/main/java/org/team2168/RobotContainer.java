// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.Autos;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.LEDs.SetBlueLED;
import org.team2168.commands.LEDs.SetGreenLED;
import org.team2168.commands.LEDs.SetRedLED;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.LEDs;

//import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.IntakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
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



  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final OI oi = OI.getInstance();
  private final LEDs leds = new LEDs();

  //private final Indexer indexer = Indexer.getInstance();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    // Configure the trigger bindings
    configureBindings();
    Logger.configureLoggingAndConfig(this, false);
  }

 
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@Link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@Link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    oi.testJoystick.ButtonA().onTrue(new SetRedLED(leds, true));
    oi.testJoystick.ButtonB().onTrue(new SetGreenLED(leds, true));
    oi.testJoystick.ButtonBack().onTrue(new SetBlueLED(leds, true));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .OnTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    oi.testJoystick.ButtonA().whileTrue(new SetIntakeSpeed(intakeRoller, .35)).onFalse(new SetIntakeSpeed(intakeRoller, 0));
    oi.testJoystick.ButtonB().whileTrue(new SetIntakeSpeed(intakeRoller, 0.45));
    oi.testJoystick.ButtonX().whileTrue(new SetIntakeSpeed(intakeRoller, .5));
    oi.testJoystick.ButtonY().whileTrue(new SetIntakeSpeed(intakeRoller, .4));

    oi.testJoystick.ButtonRightDPad().onTrue(new SetIntakePivotPosition(intakePivot, -20));
    oi.testJoystick.ButtonLeftDPad().onTrue(new SetIntakePivotPosition(intakePivot, -50));
    oi.testJoystick.ButtonDownDPad().onTrue(new SetIntakePivotPosition(intakePivot, 10));
    oi.testJoystick.ButtonUpDPad().onTrue(new SetIntakePivotPosition(intakePivot, 0));
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
