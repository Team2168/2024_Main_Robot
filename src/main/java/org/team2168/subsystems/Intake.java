// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static TalonFXHelper intakeRollerOne = new TalonFXHelper(0);
  
  /** Creates a new Intake. */
  public Intake() {
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
