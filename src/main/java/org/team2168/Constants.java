// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

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
    final int INDEXER_SENSOR = 3; //placeholder for the time being
    public static final int kDriverControllerPort = 0;
  }

  public static final class ClimberMotors{
    public static final int CLIMBER_MOTOR = 18; //we have to ask electrical if this is the number theyre using
    //public static final double UPDATE_TIME = 0; //placeholder
    //public static final int FEEDBACK_SENSOR = 0; //placeholder
  }
  public static final class Joysticks {
    public static final int DRIVER_JOYSTICK = 0; //TODO: change value
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int PID_TEST_JOYSTICK = 5;
  }

  public static final class CANDevices {
    public static final int intakePivotL = 20;
    public static final int intakePivotR = 21;
    public static final int intakeRoller = 22;
    public static final int INDEXER_MOTOR = 24;

  }

}

