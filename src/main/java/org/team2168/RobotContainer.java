// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.Autos;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
import org.team2168.commands.ShooterCommands.ShootAndControlHoodFromDistance;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.BumpShooterSpeed;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.BumpShooterSpeedDown;
import org.team2168.commands.ShooterCommands.ShooterPivot.BumpShooterAngle;
import org.team2168.commands.ShooterCommands.ShooterPivot.BumpShooterAngleDown;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;
import org.team2168.utils.F310;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter shooter = Shooter.getInstance();
  private final ShooterPivot shooterPivot = ShooterPivot.getInstance();
  private double limelightDistanceMeters = 0.0; //unknown

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final OI oi = OI.getInstance();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    oi.testJoystick.ButtonA().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPM.WHITE_LINE.shooterRPS, ShooterPivot.SHOOTING_ANGLE.WHITE_LINE.shooterAngle));
    oi.testJoystick.ButtonY().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPM.RED_LINE.shooterRPS, ShooterPivot.SHOOTING_ANGLE.RED_LINE.shooterAngle));
    oi.testJoystick.ButtonB().onTrue(new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelightDistanceMeters));
    oi.testJoystick.ButtonUpDPad().whileTrue(new BumpShooterSpeed(shooter));
    oi.testJoystick.ButtonDownDPad().whileTrue(new BumpShooterSpeedDown(shooter));
    oi.testJoystick.ButtonStart().whileTrue(new BumpShooterAngle(shooterPivot));
    oi.testJoystick.ButtonBack().whileTrue(new BumpShooterAngleDown(shooterPivot));
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
}
