// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.Drivetrain;

import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class StrafeToTagPosition extends Command {
  /** Creates a new StrafeToTagPosition. */
  private Drivetrain drivetrain;
  private Limelight limelight;
    
  private double errorToleranceAngle;
  private double limeAngle;
  private int withinThresholdLoops = 0;
  private int acceptableLoops = 10;
  
  //PID gains
  private static final double kP = 0.02168; //TODO: check gains
  private static final double kI = 0;
  private static final double kD = 0.0025;

  private static final double MINIMUM_COMMAND = 0.15;
  private static final double MAX_INTEGRAL = 1.0;

  private double strafeSpeed;
  
  private PIDController pidController;

  /**
   * Allows the robot to strafe until it is perpendicular to an AprilTag
   * @param drivetrain the drivetrain instance
   * @param limelight the limelight instance
   * @param acceptableAngleError acceptable error/offset the drivetrain can be from being perfectly perpendicular
   */
  public StrafeToTagPosition(Drivetrain drivetrain, Limelight limelight, double acceptableAngleError) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limelight = limelight;

    errorToleranceAngle = acceptableAngleError;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController = new PIDController(kP,kI, kD);
    pidController.setSetpoint(0.0); //robot's target position is being aligned/perpendicular with the AprilTag
    pidController.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);

    limelight.enableBaseCameraSettings();
    limelight.setPipeline(Limelight.Pipeline.AMPS.getPipeline());

    pidController.setTolerance(errorToleranceAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeAngle = limelight.getOffsetX();

    //robot needs to within its error tolerance for 10 loops
    //or else the counter resets
    if(Math.abs(limeAngle) < errorToleranceAngle)
      ++withinThresholdLoops;
    else 
      withinThresholdLoops = 0;

    strafeSpeed = -(pidController.calculate(limeAngle));
    
    //speed is adjusted to ensure the drivetrain will actually move
    if(limeAngle > errorToleranceAngle)
      strafeSpeed += MINIMUM_COMMAND;
    else if (limeAngle < -errorToleranceAngle)
      strafeSpeed -= MINIMUM_COMMAND;
    else 
      strafeSpeed = 0.0;

    drivetrain.drive(strafeSpeed, 0.0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return withinThresholdLoops >= acceptableLoops;
  }
}
