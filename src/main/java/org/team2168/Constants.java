// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import com.ctre.phoenix6.hardware.DeviceIdentifier;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class SHOOTER_MOTOR_CONSTANTS {
    public static final int LEFT_SHOOTER_ID = 26; //placeholder
    public static final int RIGHT_SHOOTER_ID = 27; //placeholder
    public static final int SHOOTER_PIVOT_ID = 25; //placeholder  
  }

  public static class Controllers {

    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int TEST_JOYSTICK = 5;
  
  }
}
