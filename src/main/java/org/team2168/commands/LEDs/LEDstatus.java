// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//checklist
//make sure that leds connect to certain functions of my choosing, and make sure they work sucesfully
package org.team2168.commands.LEDs;
import org.team2168.subsystems.LEDs;
import edu.wpi.first.wpilibj2.command.Command;
import org.team2168.commands.indexer.*;
import org.team2168.subsystems.Indexer;
import org.team2168.commands.Limelight.*;
import org.team2168.subsystems.Limelight;
import org.team2168.subsystems.ShooterSubsystem.Shooter;
public class LEDstatus extends Command {
  /** Creates a new LEDstatus. */
  private LEDs leds;
  private Indexer indexer;
  private Limelight limelight;
  private Shooter shooter;
  double limeErrorTolerance = 1.0; //in degrees
  public LEDstatus(LEDs leds) {
    this.leds = leds;  
    this.indexer = indexer; 
    this.limelight = limelight;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer.isNotePresent()) {
    
      leds.bluelight(true);
      
    } 
    else {
      leds.bluelight(false);
    }

    if (Math.abs(limelight.getOffsetX()) < limeErrorTolerance) {
      leds.redlight(true);
    }
      else {
        leds.redlight(false);}
        if (shooter.isAtSpeed(0.25)) {
          leds.greenlight(true);
        }
        else {
          leds.greenlight(false);
        }
      } 
      
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
