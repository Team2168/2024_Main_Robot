// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.subsystems;

import java.util.Set;

import org.team2168.Constants.PneumaticsDevices;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LED. */
   private Solenoid redLED;
  private Solenoid greenLED;
  private Solenoid blueLED;

static LEDs instance = null;
  public LEDs() {
    redLED = new Solenoid(PneumaticsDevices.MODULE_TYPE, PneumaticsDevices.RED_LIGHT);
    greenLED = new Solenoid(PneumaticsDevices.MODULE_TYPE, PneumaticsDevices.GREEN_LIGHT);
    blueLED = new Solenoid(PneumaticsDevices.MODULE_TYPE, PneumaticsDevices.BLUE_LIGHT);
  }

  private void SetAllLEDs(boolean redIsOn, boolean greenIsOn, boolean blueIsOn) {
redLED.set(redIsOn);
greenLED.set(greenIsOn);
blueLED.set(blueIsOn);
  }
  /** sets the red LED on and off
 * @param isOn whether the red light should be on (true) or off (false)
 */
  public void redlight(boolean isOn) {
    redLED.set(isOn);
  }
  /** sets the green LED on and off
 *  @param isOn whether the green light should be on (true) or off (false)
 */

  public void greenlight(boolean isOn) {
    greenLED.set(isOn);
  }
/** sets the blue LED on and off
 * @param isOn whether the blue light should be on (true) or off (false)
 */
  public void bluelight(boolean isOn) {
    blueLED.set(isOn);
  }

   @Log(name = "Is Red On?", rowIndex = 0, columnIndex = 0)
  public boolean getRedstate() {
    return redLED.get();
  }
   
 @Log(name = "Is Green On?", rowIndex = 0, columnIndex = 1)
  public boolean getGreenState() {
    return greenLED.get();
  }

    @Log(name = "Is Blue On?", rowIndex = 0, columnIndex = 0)
  public boolean getBlueState() {
    return blueLED.get();
  }
  

public static LEDs getInstance() {
  if (instance == null)
  instance = new LEDs();
  return instance;
}

 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
