package org.team2168.thirdcoast.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.TimedRobot;

public class SwerveDriveConfig {

  /**
   * PigeonIMU gyro connected to CAN bus, used for field-oriented driving. If null, field-oriented
   * driving is disabled.
   */
  public Pigeon2 gyro;

  /** Initialize with four initialized wheels, in order from wheel 0 to wheel 3.
   *  0 is front left, 1 is front right, 2 is back left, and 3 is back right
   */
  public Wheel[] wheels;

  /** Wheel base length from front to rear of robot. */
  public double length = 21.25; // inches

  /** Wheel base width from left to right of robot. */
  public double width = 14.75; // inches

  public double lengthFromCenterToWheel = 0.53975; // meters

  public double widthFromCenterToWheel = 0.37465; // meters

  /**
   * Robot period is the {@code TimedRobot} period in seconds, defaults to {@code
   * TimedRobot.kDefaultPeriod}.
   */
  public double robotPeriod = TimedRobot.kDefaultPeriod;

  /** Factor to correct gyro lag when simultaneously applying azimuth and drive. */
  public double gyroRateCoeff = 0.0;

  /** Log gyro errors, set to false if too spammy. */
  public boolean gyroLoggingEnabled = true;

  /**
   * Summarize Talon configuration errors. If false, will log error messages as each error is
   * encountered.
   */
  public boolean summarizeTalonErrors = false;
}
