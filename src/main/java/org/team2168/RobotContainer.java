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
import org.team2168.commands.Drivetrain.DriveWithLimelight;
import org.team2168.commands.Drivetrain.ZeroSwerve;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.commands.LEDs.LEDstatus;
import org.team2168.commands.LEDs.SetBlueLED;
import org.team2168.commands.LEDs.SetGreenLED;
import org.team2168.commands.LEDs.SetRedLED;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
import org.team2168.commands.ShooterCommands.ShootAndControlHoodFromDistance;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.BumpShooterSpeed;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.BumpShooterSpeedDown;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.SetShooterVelocity;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.StopFlywheel;
import org.team2168.commands.ShooterCommands.ShooterPivot.BumpShooterAngle;
import org.team2168.commands.ShooterCommands.ShooterPivot.BumpShooterAngleDown;
import org.team2168.commands.auto.DoNothing;
import org.team2168.commands.auto.LeaveStartingZone;
import org.team2168.commands.auto.OneNoteAuto;
import org.team2168.commands.auto.RotateChassisContinuous;
import org.team2168.commands.auto.TwoNoteAuto;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.LEDs;

//import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.IntakePivot;

import org.team2168.subsystems.Limelight;

import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;
import org.team2168.subsystems.Limelight;
import org.team2168.utils.F310;
import org.team2168.commands.QueueNote;
import org.team2168.commands.Drivetrain.AlignWithAmp;
import org.team2168.commands.Drivetrain.DriveWithChassisSpeedsJoystick;
import org.team2168.commands.Drivetrain.DriveWithJoystick;
import org.team2168.commands.indexer.DriveIndexeruntilNote;
import org.team2168.commands.indexer.DriveIndexeruntilnoNote;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.IntakePivot;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
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
  private final Limelight limelight = Limelight.getInstance();
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  private final OI oi = OI.getInstance();
  private final Shooter shooter = Shooter.getInstance();
  private final ShooterPivot shooterPivot = ShooterPivot.getInstance();
  private double limelightDistanceMeters = 0.0; //unknown
  private boolean brakesEnabled = false;

  @Log(name = "auto chooser", width = 2)
  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final LEDs leds = LEDs.getInstance();

  //private final Indexer indexer = Indexer.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    configureAutonomousRoutines();
    Logger.configureLoggingAndConfig(this, false);
  }

 
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@Link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@Link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // oi.testJoystick.ButtonA().onTrue(new SetRedLED(leds, true));
    // oi.testJoystick.ButtonB().onTrue(new SetGreenLED(leds, true));
    // oi.testJoystick.ButtonBack().onTrue(new SetBlueLED(leds, true));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    drivetrain.setDefaultCommand(new DriveWithJoystick(drivetrain));
    intakePivot.setDefaultCommand(new SetIntakePivotPosition(intakePivot, -120.0)); // TODO: uncomment when intakepivot works again
    leds.setDefaultCommand(new LEDstatus(leds, indexer, limelight, shooter));
    oi.driverJoystick.ButtonX().onTrue(new DriveWithLimelight(drivetrain, limelight, 0.5, true));
    oi.driverJoystick.ButtonLeftBumper().onTrue(new DriveWithJoystick(drivetrain)); // cancels drivewithlimelight command
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .OnTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // drivetrain.setDefaultCommand(new DriveWithChassisSpeedsJoystick(drivetrain));

    oi.operatorJoystick.ButtonA().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle));
    oi.operatorJoystick.ButtonY().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.STARTING_ZONE_LINE.shooterRPS, ShooterPivot.SHOOTING_ANGLE.STARTING_ZONE_LINE.shooterAngle));
    oi.operatorJoystick.ButtonB().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_AMP.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_AMP.shooterAngle));
    oi.operatorJoystick.ButtonX().onTrue(new StopFlywheel(shooter));
    // oi.operatorJoystick.ButtonB().whileTrue(new DriveIndexeruntilNote(indexer, () -> 0.6));
    oi.operatorJoystick.ButtonStart().onTrue(new BumpShooterAngle(shooterPivot));
    oi.operatorJoystick.ButtonBack().onTrue(new BumpShooterAngleDown(shooterPivot));
    oi.operatorJoystick.ButtonLeftBumper().whileTrue(new ContinuousNoteQueue(indexer, intakeRoller))
                                          .whileTrue(new SetIntakePivotPosition(intakePivot, -12.5))
                                          .whileFalse(new SetIntakePivotPosition(intakePivot, -120.0)); // TODO: uncomment when intake pivot is brought back
    //oi.operatorJoystick.ButtonLeftBumper().whileTrue(new RepeatCommand(new QueueNote(intakeRoller, indexer))); // TODO: test

    oi.driverJoystick.ButtonBack().onTrue(new AlignWithAmp(drivetrain, limelight));
    oi.driverJoystick.ButtonStart().whileTrue(new SetIntakeSpeed(intakeRoller, -0.5));

    oi.operatorJoystick.ButtonRightBumper().whileTrue(new DriveIndexeruntilnoNote(indexer, () -> 1.0));

    //Testing LEDs
    // oi.testJoystick.ButtonB().whileTrue(new SetRedLED(leds, true))
    //                          .onFalse(new SetRedLED(leds, false));

    // oi.testJoystick.ButtonA().whileTrue(new SetGreenLED(leds, true))
    //                          .onFalse(new SetGreenLED(leds, false));

    // oi.testJoystick.ButtonX().whileTrue(new SetBlueLED(leds, true))
    //                          .onFalse(new SetBlueLED(leds, false));
    oi.testJoystick.ButtonA().onTrue(new BumpShooterSpeed(shooter));
    oi.testJoystick.ButtonB().onTrue(new BumpShooterSpeedDown(shooter));

  }

  public void configureAutonomousRoutines() {
    autoChooser.setDefaultOption("Do Nothing", new DoNothing());
    autoChooser.addOption("One Note", new OneNoteAuto(drivetrain, intakePivot, indexer, shooter, shooterPivot, limelight, leds));
    autoChooser.addOption("Two Note", new TwoNoteAuto(drivetrain, intakeRoller, intakePivot, indexer, shooter, shooterPivot, limelight, leds));
    autoChooser.addOption("Drive Back", new LeaveStartingZone(drivetrain, intakePivot));
    autoChooser.addOption("Rotate Chassis Continuous", new RotateChassisContinuous(drivetrain));

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    var auto = autoChooser.getSelected();
    if (auto == null) {
      return new DoNothing();
    }
    else {
      return autoChooser.getSelected();
    }
  }

  public boolean getBrakesEnabled() {
    return brakesEnabled;
  }

  @Config(name = "kickable robot?", width = 2)
  public void setBrakesEnabled(boolean enabled) {
    brakesEnabled = enabled;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
}
