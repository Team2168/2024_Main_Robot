// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
  public static final double LOOP_TIME_SEC = 0.02;
  public static final class PneumaticsDevices {
  public static final int RED_LIGHT = 5;
  public static final int GREEN_LIGHT = 7;
  public static final int BLUE_LIGHT = 12;

    public static final PneumaticsModuleType MODULE_TYPE = PneumaticsModuleType.REVPH;
  }
  public static class OperatorConstants {
    final int INDEXER_SENSOR = 3; //placeholder for the time being
    public static final int kDriverControllerPort = 0;
  }

  public static class CANDevices {

    // ARBRITRARY MOTORS
      
      public static final int DRIVE_MOTOR_FL = 2;
      public static final int DRIVE_MOTOR_FR = 13;
      public static final int DRIVE_MOTOR_BL = 12;
      public static final int DRIVE_MOTOR_BR = 3;
      
      public static final int[] DRIVE_MOTORS = {DRIVE_MOTOR_FL, DRIVE_MOTOR_FR, DRIVE_MOTOR_BL, DRIVE_MOTOR_BR};
      
      public static final int AZIMUTH_MODULE_FL = 11;
      public static final int AZIMUTH_MODULE_FR = 1;
      public static final int AZIMUTH_MODULE_BL = 0;
      public static final int AZIMUTH_MODULE_BR = 6;
  
      public static final int[] AZIMUTH_MODULES = {AZIMUTH_MODULE_FL, AZIMUTH_MODULE_FR, AZIMUTH_MODULE_BL, AZIMUTH_MODULE_BR};
  
      public static final int CANCODER_0_CAN_ID = 14;
      public static final int CANCODER_1_CAN_ID = 4;
      public static final int CANCODER_2_CAN_ID = 8;
      public static final int CANCODER_3_CAN_ID = 7;
  
      public static final int[] CANCODER_ID = {CANCODER_0_CAN_ID, CANCODER_1_CAN_ID, CANCODER_2_CAN_ID, CANCODER_3_CAN_ID}; 
  
      public static final int PIGEON_CAN_ID = 17;

      public static final int intakePivotL = 20;
      public static final int intakePivotR = 21;
      public static final int intakeRoller = 22;
      public static final int INDEXER_MOTOR = 24;
      public static final int LEFT_SHOOTER_ID = 26; //placeholder
      public static final int RIGHT_SHOOTER_ID = 27; //placeholder
      public static final int SHOOTER_PIVOT_ID = 25; //placeholder  
    }

  public static class Controllers {

    public static final int DRIVER_JOYSTICK = 0;
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int TEST_JOYSTICK = 5;
  
  }

  public static class Drivetrain {
    public static final double kpDriveVel = 3.0; // needs testing
    public static final double kpAngularVel = 1.0; // needs to be tested
    public static final double kdAngularVel = 0.0;
    public static final double kiAngularVel = 0.0;
    public static double ksVolts;
    public static double kvVoltSecondsPerMeter;
    public static double kaVoltSecondsSquaredPerMeter;
  }
  public static final class Joysticks {
    public static final int DRIVER_JOYSTICK = 0; //TODO: change value
    public static final int OPERATOR_JOYSTICK = 1;
    public static final int TEST_JOYSTICK = 5;
  
  }

  public static class RobotMeasurements {
    public static final double LIMELIGHT_TO_CENTER_IN = 0.5;
    public static final double LIMELIGHT_TO_CENTER_M = 0.0127;

    public static final double TOP_TO_LIMELIGHT_IN = 20.5;
    public static final double TOP_TO_LIMELIGHT_M = 0.5207;
  }

}

