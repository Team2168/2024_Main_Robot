// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;

import org.team2168.subsystems.Climber;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberEvenTiltOut extends Command {
  /** Creates a new ClimberEvenTiltOut. */
  Climber climberLeft;
  Climber climberRight;
  double gyroTilt;
  double kMaxVel;

  public ClimberEvenTiltOut(Climber climberLeft, Climber climberRight, double gyroTilt, double kMaxVel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberLeft = climberLeft;
    this.climberRight = climberRight;
    this.gyroTilt = gyroTilt;
    this.kMaxVel = kMaxVel;

    addRequirements(climberLeft, climberRight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (gyroTilt < 0){
      climberLeft.setLeftMaxVel(kMaxVel);
      climberLeft.setLeftToZero();
      climberRight.setRightMaxVel(kMaxVel + Climber.inchesToRotations(1));
      climberRight.setRightToZero();
      climberRight.setRightMaxVel(0);
    }
    else if (gyroTilt > 0){
      climberRight.setRightMaxVel(kMaxVel);
      climberRight.setRightToZero();
      climberLeft.setLeftMaxVel(kMaxVel + Climber.inchesToRotations(1));
      climberLeft.setLeftToZero();
      climberLeft.setLeftMaxVel(kMaxVel);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (gyroTilt == 0);
  }
}