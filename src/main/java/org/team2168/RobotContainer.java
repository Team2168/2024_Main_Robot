// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import javax.sql.ConnectionPoolDataSource;

import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.Autos;
import org.team2168.commands.ContinuousNoteQueue;
import org.team2168.commands.ExampleCommand;
import org.team2168.commands.Drivetrain.DriveWithJoystick;
import org.team2168.commands.Drivetrain.ZeroSwerve;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
import org.team2168.commands.ShooterCommands.ShootAndControlHoodFromDistance;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.BumpShooterSpeed;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.BumpShooterSpeedDown;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.SetShooterVelocity;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.StopFlywheel;
import org.team2168.commands.ShooterCommands.ShooterPivot.BumpShooterAngle;
import org.team2168.commands.ShooterCommands.ShooterPivot.BumpShooterAngleDown;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;
import org.team2168.subsystems.Limelight;
import org.team2168.utils.F310;
import org.team2168.commands.QueueNote;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.indexer.DriveIndexeruntilNote;
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
  static RobotContainer instance = null;

  private final IntakeRoller intakeRoller = IntakeRoller.getInstance();
  private final IntakePivot intakePivot = IntakePivot.getInstance();
  private final Indexer indexer = Indexer.getInstance();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Limelight limelight = new Limelight();
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  private final OI oi = OI.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final ShooterPivot shooterPivot = ShooterPivot.getInstance();
  private double limelightDistanceMeters = 0.0; //unknown

  //private final Indexer indexer = Indexer.getInstance();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final F310 driverJoystick = oi.driverJoystick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    Logger.configureLoggingAndConfig(this, false);
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
    
    drivetrain.setDefaultCommand(new DriveWithJoystick(drivetrain));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // oi.driverJoystick.ButtonX().onTrue(new ZeroSwerve(drivetrain));

    oi.operatorJoystick.ButtonA().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle));
    oi.operatorJoystick.ButtonY().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.STARTING_ZONE_LINE.shooterRPS, ShooterPivot.SHOOTING_ANGLE.STARTING_ZONE_LINE.shooterAngle));
    oi.operatorJoystick.ButtonB().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_AMP.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_AMP.shooterAngle));
    oi.operatorJoystick.ButtonX().onTrue(new StopFlywheel(shooter));
    oi.testJoystick.ButtonRightBumper().onTrue(new BumpShooterSpeed(shooter));
    oi.testJoystick.ButtonLeftBumper().onTrue(new BumpShooterSpeedDown(shooter));
    oi.operatorJoystick.ButtonStart().onTrue(new BumpShooterAngle(shooterPivot));
    oi.operatorJoystick.ButtonBack().onTrue(new BumpShooterAngleDown(shooterPivot));
    // oi.testJoystick.ButtonX().whileTrue(new SetIntakeSpeed(intakeRoller, .5));
    // oi.testJoystick.ButtonY().whileTrue(new SetIntakeSpeed(intakeRoller, .4));
    oi.operatorJoystick.ButtonLeftBumper().whileTrue(new ContinuousNoteQueue(indexer, intakeRoller));
    // oi.operatorJoystick.ButtonLeftBumper().whileTrue(new RepeatCommand(new QueueNote(intakeRoller, indexer))); // TODO: test
    oi.operatorJoystick.ButtonLeftBumper().whileTrue(new SetIntakePivotPosition(intakePivot, 0.0)).onFalse(new SetIntakePivotPosition(intakePivot, -120.0));
    oi.operatorJoystick.ButtonRightBumper().whileTrue(new DriveIndexeruntilnoNote(indexer, () -> 0.75));

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
