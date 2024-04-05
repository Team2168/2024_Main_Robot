// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import org.team2168.subsystems.IntakePivot;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.Limelight.Pipeline;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  Timer timer = new Timer();

  private IntakePivot intakePivot;
  private RobotContainer m_robotContainer;
  public Limelight limelight;
  private Drivetrain drivetrain;

  private Alliance lastAllianceReport = Alliance.Blue;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    limelight = Limelight.getInstance();
    drivetrain = Drivetrain.getInstance();
    intakePivot = IntakePivot.getInstance();

    limelight.enableVision(true);
    FollowPathHolonomic.warmupCommand().schedule(); // attempts to get rid of random error occuring on robot power-on
    PathfindHolonomic.warmupCommand().schedule();
    intakePivot.resetIntakeEncodersToStow(); // only reset intake pivot among first time
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    Logger.updateEntries();
    SmartDashboard.updateValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    timer.reset();
    timer.start();
  }

  @Override
  public void disabledPeriodic() {
    // DriverStation.refreshData();
    // drivetrain.updatePathInvert();

    if (DriverStation.getAlliance().isPresent() && lastAllianceReport != DriverStation.getAlliance().get()) {
      m_robotContainer.configureAutonomousRoutines();
      m_robotContainer.configureBindings();
      lastAllianceReport = DriverStation.getAlliance().get();
    }
    if (timer.hasElapsed(1)) {
      // intakePivotOne needs to have an invert of true, intakePivotTwo needs to have an invert of false
      if (!intakePivot.getIntakeOneInvert() || intakePivot.getIntakeTwoInvert()) {
        intakePivot.setInvertConfiguration();
      }
      timer.reset();
    }
    //drivetrain.setMotorsBrake(m_robotContainer.getBrakesEnabled());
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    drivetrain.driveToChassisSpeed(new ChassisSpeeds(0.0, 0.0, 0.0));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    limelight.enableBaseCameraSettings();
    limelight.setPipeline(Pipeline.ALL_APRIL_TAGS.pipelineValue);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // intakePivot.setIntakePivotPosition(-120.0);
    drivetrain.setMotorsBrake(true);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    drivetrain.driveToChassisSpeed(new ChassisSpeeds(0.0, 0.0, 0.0)); // cancels speed remaining from autonomous
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    limelight.enableBaseCameraSettings();
    limelight.setPipeline(Pipeline.ALL_APRIL_TAGS.pipelineValue);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // intakePivot.setIntakePivotPosition(-120.0);

    drivetrain.setMotorsBrake(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  @Override
  public void driverStationConnected() {
    m_robotContainer.configureAutonomousRoutines();
    m_robotContainer.configureBindings();
  }
}
