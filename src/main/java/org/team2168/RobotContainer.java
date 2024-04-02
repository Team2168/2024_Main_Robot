// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.Constants.OperatorConstants;
import org.team2168.commands.ContinuousNoteQueue;
import org.team2168.commands.Drivetrain.DriveWithJoystick;
import org.team2168.commands.Drivetrain.DriveWithLimelight;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.ExampleSubsystem;
import org.team2168.commands.LEDs.LEDstatus;
import org.team2168.commands.intakePivot.SetIntakePivotPosition;
import org.team2168.commands.intakerRoller.SetIntakeSpeed;
import org.team2168.commands.ShooterCommands.ControlShooterAndHood;
import org.team2168.commands.ShooterCommands.ShootAndControlHoodFromDistance;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.BumpShooterSpeed;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.BumpShooterSpeedDown;
import org.team2168.commands.ShooterCommands.ShooterFlywheel.StopFlywheel;
import org.team2168.commands.ShooterCommands.ShooterPivot.BumpShooterAngle;
import org.team2168.commands.ShooterCommands.ShooterPivot.BumpShooterAngleDown;
import org.team2168.commands.auto.DoNothing;
import org.team2168.commands.auto.FasterCloseFourNote;
import org.team2168.commands.auto.FourNoteClose;
import org.team2168.commands.auto.FourNoteFar;
import org.team2168.commands.auto.LeaveStartingZone;
import org.team2168.commands.auto.OneNoteAuto;
import org.team2168.commands.auto.PathFindToAmp;
import org.team2168.commands.auto.PathFindToChain;
import org.team2168.commands.auto.PathFindToSpeaker;
import org.team2168.commands.auto.RotateChassisContinuous;
import org.team2168.commands.auto.ThreeNoteAltSide;
import org.team2168.commands.auto.TwoNoteAuto;
import org.team2168.commands.auto.WPIThreeNote;
import org.team2168.subsystems.LEDs;

//import org.team2168.subsystems.Indexer;
import org.team2168.subsystems.IntakeRoller;
import org.team2168.subsystems.IntakePivot;

import org.team2168.subsystems.Limelight;

import org.team2168.subsystems.ShooterSubsystem.Shooter;
import org.team2168.subsystems.ShooterSubsystem.ShooterPivot;
import org.team2168.utils.SwervePathUtil;
import org.team2168.utils.SwervePathUtil.InitialPathState;
import org.team2168.commands.QueueNote;
import org.team2168.commands.Drivetrain.AlignWithAmp;
import org.team2168.commands.Drivetrain.DriveToHeading;
import org.team2168.commands.indexer.DriveIndexer;
import org.team2168.commands.indexer.DriveIndexeruntilNote;
import org.team2168.commands.indexer.DriveIndexeruntilnoNote;
import org.team2168.subsystems.Indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
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
  public void configureBindings() {
    // oi.testJoystick.ButtonA().onTrue(new SetRedLED(leds, true));
    // oi.testJoystick.ButtonB().onTrue(new SetGreenLED(leds, true));
    // oi.testJoystick.ButtonBack().onTrue(new SetBlueLED(leds, true));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    drivetrain.setDefaultCommand(new DriveWithJoystick(drivetrain));
    intakePivot.setDefaultCommand(new SetIntakePivotPosition(intakePivot, -120.0)); // TODO: uncomment when intakepivot works again
    leds.setDefaultCommand(new LEDstatus(leds, indexer, limelight, shooter));
    
    // oi.driverJoystick.ButtonX().onTrue(new DriveWithLimelight(drivetrain, limelight, 0.5, true));
    // oi.driverJoystick.ButtonLeftBumper().onTrue(new DriveWithJoystick(drivetrain)); // cancels drivewithlimelight command
    // oi.driverJoystick.ButtonBack().onTrue(new AlignWithAmp(drivetrain, limelight));

    
    // oi.driverJoystick.ButtonX().onTrue(new PathFindToAmp(drivetrain));  
    //oi.operatorJoystick.ButtonLeftBumper().whileTrue(new RepeatCommand(new QueueNote(intakeRoller, indexer))); // TODO: test

    //reset 
    oi.operatorJoystick1.ButtonLeftStick().onTrue(new StopFlywheel(shooter));
    // amp
    oi.operatorJoystick1.ButtonA().onTrue(
      new ControlShooterAndHood(shooter, shooterPivot, 
                                Shooter.SHOOTING_RPS.UP_AGAINST_AMP.shooterRPS, 
                                ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_AMP.shooterAngle));
   // variable
    oi.operatorJoystick1.ButtonB().onTrue(
      new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight));
    // close
    oi.operatorJoystick1.ButtonX().onTrue(
      new ControlShooterAndHood(shooter, shooterPivot, 
                                Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS,
                                ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle));
    // shoot
    oi.operatorJoystick1.ButtonY().whileTrue(new DriveIndexeruntilnoNote(indexer, () -> 1.0));
    // source intake
    oi.operatorJoystick1.ButtonLeftBumper().whileTrue(new DriveIndexeruntilNote(indexer, () -> 0.75))
                                           .whileTrue(new SetIntakeSpeed(intakeRoller, -0.5));
                                          //  .whileFalse(new DriveIndexer(indexer, () -> 0.0));
    //floor intake
    oi.operatorJoystick1.ButtonRightStick()
      .whileTrue(new ContinuousNoteQueue(indexer, intakeRoller))
      .whileTrue(new SetIntakePivotPosition(intakePivot, -11.0))
      .whileFalse(new SetIntakePivotPosition(intakePivot, -120.0))
      .whileFalse(new DriveIndexeruntilNote(indexer, () -> 0.75).withTimeout(3.0)); // continues running indexer to intake a stuck note after intake is lifted
    // spit
    oi.operatorJoystick1.ButtonBack()
      .whileTrue(new SetIntakeSpeed(intakeRoller, -1.0))
      .whileTrue(new DriveIndexer(indexer, ()->-0.75));
    // stop shooter
    oi.operatorJoystick1.ButtonStart().onTrue(new StopFlywheel(shooter));
    //toggle climb
    //oi.operatorJoystick2.ButtonA().toggleOnTrue(new climber thing)
                                  //.toggleOnFalse(turn climber off);
    //shooter angle bumping
    oi.operatorJoystick2.ButtonX().onTrue(new BumpShooterAngle(shooterPivot));
    oi.operatorJoystick2.ButtonY().onTrue(new BumpShooterAngleDown(shooterPivot));
    // shooting speed bumping
    oi.operatorJoystick2.ButtonLeftBumper().whileTrue(new DriveIndexeruntilNote(indexer, () -> 0.75))
                                           .whileTrue(new SetIntakeSpeed(intakeRoller, -0.5));
    oi.operatorJoystick2.ButtonRightBumper().onTrue(new BumpShooterSpeedDown(shooter));

    //old operator button bindings (for F310)
    // oi.operatorJoystick.ButtonA().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_SPEAKER.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_SPEAKER.shooterAngle));
    // oi.operatorJoystick.ButtonY().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.STARTING_ZONE_LINE.shooterRPS, ShooterPivot.SHOOTING_ANGLE.STARTING_ZONE_LINE.shooterAngle));
    // oi.operatorJoystick.ButtonB().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.RED_LINE.shooterRPS, ShooterPivot.SHOOTING_ANGLE.RED_LINE.shooterAngle));
    oi.operatorJoystick.ButtonB().onTrue(new ShootAndControlHoodFromDistance(shooter, shooterPivot, limelight));
    oi.operatorJoystick.ButtonX().onTrue(new StopFlywheel(shooter));
    oi.operatorJoystick.ButtonA().onTrue(new ControlShooterAndHood(shooter, shooterPivot, Shooter.SHOOTING_RPS.UP_AGAINST_AMP.shooterRPS, ShooterPivot.SHOOTING_ANGLE.UP_AGAINST_AMP.shooterAngle));
    // oi.operatorJoystick.ButtonB().whileTrue(new DriveIndexeruntilNote(indexer, () -> 0.6));
    oi.operatorJoystick.ButtonY().whileTrue(new DriveIndexeruntilNote(indexer, () -> 0.75));
    oi.operatorJoystick.ButtonY().whileTrue(new SetIntakeSpeed(intakeRoller, -0.5));
    oi.operatorJoystick.ButtonStart().onTrue(new BumpShooterAngle(shooterPivot));
    oi.operatorJoystick.ButtonBack().onTrue(new BumpShooterAngleDown(shooterPivot));
    oi.operatorJoystick.ButtonLeftBumper().whileTrue(new ContinuousNoteQueue(indexer, intakeRoller))
                                          .whileTrue(new SetIntakePivotPosition(intakePivot, -12.5))
                                          .whileFalse(new DriveIndexeruntilNote(indexer, () -> 0.75).withTimeout(3.0))
                                          .whileFalse(new SetIntakePivotPosition(intakePivot, -120.0)); // TODO: uncomment when intake pivot is brought back
    oi.operatorJoystick.ButtonRightBumper().whileTrue(new DriveIndexeruntilnoNote(indexer, () -> 1.0)); // TODO: test

    //oi.driverJoystick.ButtonBack().onTrue(new AlignWithAmp(drivetrain, limelight));
    //oi.driverJoystick.ButtonX().whileTrue(new DriveToHeading(drivetrain, 90.0).withTimeout(1.5)); // TO TEST
    oi.driverJoystick.ButtonBack().whileTrue(new DriveWithLimelight(drivetrain, limelight, 1.0, true));
    oi.driverJoystick.ButtonStart().onTrue(new DriveWithJoystick(drivetrain));
    oi.driverJoystick.ButtonLeftStick().onTrue(new PathFindToAmp(drivetrain)); // TODO: troubleshoot
    oi.driverJoystick.ButtonRightStick().onTrue(new PathFindToSpeaker(drivetrain));
    // oi.driverJoystick.ButtonRightStick().onTrue(new PathFindToChain(drivetrain));
    //oi.driverJoystick.ButtonStart().whileTrue(new DriveWithJoystick(drivetrain)); // TODO: add button binding for amp alignment and climber alignment


    oi.testJoystick.ButtonA().onTrue(new BumpShooterSpeed(shooter));
    oi.testJoystick.ButtonB().onTrue(new BumpShooterSpeedDown(shooter));

  }

  public void configureAutonomousRoutines() {
    // autoChooser = new SendableChooser<Command>(); // reinitializes autoChooser upon configuration

    autoChooser.setDefaultOption("Do Nothing", new DoNothing());
    autoChooser.addOption("One Note", new OneNoteAuto(drivetrain, intakePivot, indexer, shooter, shooterPivot, limelight, leds));
    autoChooser.addOption("Two Note", new TwoNoteAuto(drivetrain, intakeRoller, intakePivot, indexer, shooter, shooterPivot, limelight, leds));
    autoChooser.addOption("Drive Back", new LeaveStartingZone(drivetrain, intakePivot));
    autoChooser.addOption("Rotate Chassis Continuous", new RotateChassisContinuous(drivetrain));
    autoChooser.addOption("Close 4 Note", new FourNoteClose(drivetrain, intakeRoller, intakePivot, indexer, shooter, shooterPivot, limelight, leds));
    autoChooser.addOption("Far 4 Note", new FourNoteFar(drivetrain, intakeRoller, intakePivot, indexer, shooter, shooterPivot, limelight, leds));
    autoChooser.addOption("Rotational Accuracy Auto", SwervePathUtil.getPathCommand("Rotational_Accuracy_Test", drivetrain, InitialPathState.DISCARDHEADING));
    autoChooser.addOption("3 Note Alt", new ThreeNoteAltSide(drivetrain, intakeRoller, intakePivot, indexer, shooter, shooterPivot, limelight, leds));
    autoChooser.addOption("WPI 3 Note", new WPIThreeNote(drivetrain, intakeRoller, intakePivot, indexer, shooter, shooterPivot, limelight, leds));
    autoChooser.addOption("Faster Close 4 Note", new FasterCloseFourNote(drivetrain, intakeRoller, intakePivot, indexer, shooter, shooterPivot, limelight, leds));

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

  @Log(name = "is alliance red?")
  public boolean isAllianceRed() {
    if (DriverStation.getAlliance().isPresent()) {
      return (DriverStation.getAlliance().get() == Alliance.Red);
    }
    else {
      return false;
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
}
