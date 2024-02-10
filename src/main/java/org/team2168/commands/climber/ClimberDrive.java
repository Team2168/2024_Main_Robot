// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.commands.climber;
import java.util.function.DoubleSupplier;

import org.team2168.subsystems.ClimberLeft;
import org.team2168.subsystems.ClimberRight;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberDrive extends InstantCommand {
  ClimberLeft climberLeft;
  ClimberRight climberRight;
  DoubleSupplier speed;

  public ClimberDrive(ClimberLeft climberLeft, ClimberRight climberRight, DoubleSupplier s) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberLeft = climberLeft;
    this.climberRight = climberRight;
    speed = s;

    addRequirements(climberLeft, climberRight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //elevator.setPercentOutput(elevatorPercentOutput);
  // if (elevator.isInRange()) {
    climberLeft.setVolt(speed.getAsDouble());
    climberRight.setVolt(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    climberLeft.setVolt(0);
    climberRight.setVolt(0);
}
}
