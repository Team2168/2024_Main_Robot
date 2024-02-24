/**
 * This code was written primarily as a part of FRC Team 2767's Third Coast Library
 * It has been adapted to be used by 2168
 */
package org.team2168.thirdcoast.swerve;

import java.util.Objects;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import org.team2168.thirdcoast.swerve.SwerveDrive.DriveMode;

/**
 * Controls a swerve drive wheel azimuth and drive motors.
 *
 * <p>
 * The swerve-drive inverse kinematics algorithm will always calculate
 * individual wheel angles as
 * -0.5 to 0.5 rotations, measured clockwise with zero being the straight-ahead
 * position. Wheel
 * speed is calculated as 0 to 1 in the direction of the wheel angle.
 *
 * <p>
 * This class will calculate how to implement this angle and drive direction
 * optimally for the
 * azimuth and drive motors. In some cases it makes sense to reverse wheel
 * direction to avoid
 * rotating the wheel azimuth 180 degrees.
 *
 * <p>
 * Hardware assumed by this class includes a CTRE magnetic encoder on the
 * azimuth motor and no
 * limits on wheel azimuth rotation. Azimuth Talons have an ID in the range 0-3
 * with corresponding
 * drive Talon IDs in the range 10-13.
 */
public class Wheel {
  private static final double AZIMUTH_GEAR_RATIO = (8.0 / 32.0) * (32.0 / 24.0); // (60.0/10.0) * (45.0/15.0); //
                                                                                 // defined as module input/motor
                                                                                 // output; placeholder
  private static final double DRIVE_GEAR_RATIO = 7.13 / 1; // (60.0/15.0) * (20.0/24.0) * (38.0/18.0);
  private static final double DRIVE_CIRCUMFERENCE_FT = ((Math.PI * 4.0) / 12.0);
  private static final double DRIVE_CIRCUMFERENCE_M = 0.3048 * DRIVE_CIRCUMFERENCE_FT;
  private static final int INTERNAL_ENCODER_TICKS = 2048;
  private static final int EXTERNAL_ENCODER_TICKS = 4096;
  private static final double AZIMUTH_ERROR_TOLERANCE_DEG = 2.0;
  private static final double TICKS_PER_DEGREE_AZIMUTH = ((1.0 / 360.0) * EXTERNAL_ENCODER_TICKS);
  private static final double TICKS_PER_DEGREE_DW = ((1.0 / 360.0) * DRIVE_GEAR_RATIO * INTERNAL_ENCODER_TICKS);
  public static final double ROTS_PER_FOOT_DW = ((1.0 / DRIVE_CIRCUMFERENCE_FT) * DRIVE_GEAR_RATIO); // TODO: check math?
  private static final double EXTERNAL_ENCODER_TICKS_PER_REV = 360.0 * TICKS_PER_DEGREE_AZIMUTH;
  private static final double FREE_SPEED_RPM = 6380;
  private static final double FREE_SPEED_RPS = FREE_SPEED_RPM / 60.0;
  private static final double DRIVE_SETPOINT_MAX = FREE_SPEED_RPS / 10.0; // rotations/100 ms, new phoenix 6 libs
  private final TalonFX driveTalon;
  private final TalonFX azimuthTalon;
  private DutyCycleOut percentOutDutyCycle;
  private VelocityVoltage velocityVoltage;
  private MotionMagicVoltage motionMagicVoltage;
  protected DoubleConsumer driver;
  private boolean isInverted = false;
  private static final int PRIMARY_PID = 0;
  private static final int AUX_PID = 1; // the auxiliary pid loop on the CTRE motor controllers

  /**
   * This constructs a wheel with supplied azimuth and drive talons.
   *
   * <p>
   * Wheels will scale closed-loop drive output to {@code DRIVE_SETPOINT_MAX}. For
   * example, if
   * closed-loop drive mode is tuned to have a max usable output of 10,000 ticks
   * per 100ms, set this
   * to 10,000 and the wheel will send a setpoint of 10,000 to the drive talon
   * when wheel is set to
   * max drive output (1.0).
   *
   * @param azimuth the configured azimuth TalonFX
   * @param drive   the configured drive TalonFX
   */
  public Wheel(TalonFX azimuth, TalonFX drive) {
    azimuthTalon = Objects.requireNonNull(azimuth);
    driveTalon = Objects.requireNonNull(drive);

    percentOutDutyCycle = new DutyCycleOut(0.0);
    motionMagicVoltage = new MotionMagicVoltage(0.0);
    velocityVoltage = new VelocityVoltage(0.0);

  }

  /**
   * This method calculates the optimal driveTalon settings and applies them.
   *
   * @param azimuth -0.5 to 0.5 rotations, measured clockwise with zero being the
   *                wheel's zeroed
   *                position
   * @param drive   0 to 1.0 in the direction of the wheel azimuth
   */
  public void set(double azimuth, double drive) {
    // don't reset wheel azimuth direction to zero when returning to neutral
    // if (drive == 0) {
    // driver.accept(0d);
    // return;
    // }
    // azimuth *= -EXTERNAL_ENCODER_TICKS_PER_REV; // flip azimuth, hardware configuration dependent (everything uses encoder rotations now)
    azimuth *= -1.0;

    double azimuthPosition = azimuthTalon.getPosition().getValue();
    double azimuthError = azimuth - azimuthPosition;

    // minimize azimuth rotation, reversing drive if necessary
    isInverted = Math.abs(azimuthError) > (0.25 + degToRotations(AZIMUTH_ERROR_TOLERANCE_DEG));
    if (isInverted) {
      azimuthError -= Math.copySign(0.5, azimuthError);
      drive = -drive;
    }

    azimuthTalon.setControl(motionMagicVoltage.withPosition((azimuthPosition + azimuthError)));
    driver.accept(drive);
  }

  public void setWithModuleState(SwerveModuleState modState) {
    SwerveModuleState optimModState = SwerveModuleState.optimize(modState, new Rotation2d(getAzimuthPosition() * 2 * Math.PI)); // optimal module state
    driveTalon.set(((optimModState.speedMetersPerSecond / DRIVE_CIRCUMFERENCE_M) / DRIVE_SETPOINT_MAX)); // returns m/s drive speed to percentage
    azimuthTalon.setControl(new MotionMagicVoltage(optimModState.angle.getRotations()));
  }

  /**
   * Set azimuth motor to encoder position.
   *
   * @param position position in encoder ticks.
   */
  public void setAzimuthMotorPosition(double position) {
    azimuthTalon.setControl(motionMagicVoltage.withPosition(position));
  }

  /**
   * Set module heading
   *
   * @param position position in motor ticks
   */
  public void setAzimuthPosition(int position) {
    setAzimuthMotorPosition((int) (position / AZIMUTH_GEAR_RATIO));
  }

  public void disableAzimuth() {
    azimuthTalon.disable();
  }

  /**
   * Set the operating mode of the wheel's drive motors. In this default wheel
   * implementation {@code
   * OPEN_LOOP} and {@code TELEOP} are equivalent and {@code CLOSED_LOOP},
   * {@code TRAJECTORY} and
   * {@code AZIMUTH} are equivalent.
   *
   * <p>
   * In closed-loop modes, the drive setpoint is scaled by the drive Talon {@code
   * DRIVE_SETPOINT_MAX} parameter.
   *
   * <p>
   * This method is intended to be overridden if the open or closed-loop drive
   * wheel drivers need
   * to be customized.
   *
   * @param driveMode the desired drive mode
   */
  public void setDriveMode(DriveMode driveMode) {
    switch (driveMode) {
      case OPEN_LOOP:
      case TELEOP:
        driver = (setpoint) -> driveTalon.setControl(percentOutDutyCycle.withOutput(setpoint));
        break;
      case CLOSED_LOOP:
      case TRAJECTORY:
      case AZIMUTH:
        driver = (setpoint) -> driveTalon.setControl(velocityVoltage.withVelocity(setpoint * DRIVE_SETPOINT_MAX));
        break;
    }
  }

  /**
   * Stop azimuth and drive movement. This resets the azimuth setpoint and
   * relative encoder to the
   * current position in case the wheel has been manually rotated away from its
   * previous setpoint.
   */
  public void stop() {
    azimuthTalon.setControl(motionMagicVoltage.withPosition(azimuthTalon.getPosition().getValue()));
    driver.accept(0d);
  }

  /**
   * Set the azimuthTalon encoder relative to wheel zero alignment position. For
   * example, if current
   * absolute encoder = 0 and zero setpoint = 2767, then current relative setpoint
   * = -2767.
   *
   * <pre>
   *
   * relative:  -2767                               0
   *           ---|---------------------------------|-------
   * absolute:    0                               2767
   *
   * </pre>
   *
   * @param zero zero setpoint, absolute encoder position (in ticks) where wheel
   *             is zeroed.
   */
  public void setAzimuthZero(double zero) {
    // double azimuthSetpoint = (double) getAzimuthPosition() - zero;
    // ErrorCode err =
    // azimuthTalon.setSelectedSensorPosition(externalToInternalTicks(azimuthSetpoint),
    // primaryPID, 10);
    // Errors.check(err, logger);
    System.out.println("magnetOffset: " + zero);
    System.out.println("current relative pos: " + getAzimuthPosition());
    // azimuthTalon.setPosition(-azimuthSetpoint);
    azimuthTalon.setControl(motionMagicVoltage.withPosition(0.0));
    // System.out.println("SETPOINT: " + -azimuthSetpoint);
  }

  /**
   * Takes in a number of ticks from the external encoder of a module, and
   * estamates a number of internal
   * ticks based off the number
   * 
   * @param externalTicks a number of ticks from the external encoder
   * @return a proportional number of estamated internal ticks
   */
  // public static int externalToInternalTicks(int externalTicks) {
  // return (int) Math.round((double) externalTicks*((double)
  // INTERNAL_ENCODER_TICKS/(double) EXTERNAL_ENCODER_TICKS)*AZIMUTH_GEAR_RATIO);
  // }

  /**
   * Takes in a number of degrees that we want to rotate the azimuth motor by and
   * converts it to the number of ticks
   * the internal encoder should move by
   *
   * @param degrees number of degrees the wheel needs to rotate
   * @return the number of ticks the internal encoder should rotate in order to
   *         rotate the azimuth motor
   */
  // public static int degreesToTicksAzimuth(double degrees) {
  //   return (int) (degrees * TICKS_PER_DEGREE_AZIMUTH);
  // }

  /**
   * Converts degrees into Phoenix 6 new sensor units
   * 
   * @param deg the number of degrees to be converted
   * @return conversion of degree values to rotations
   */
  public double degToRotations(double deg) {
    return (deg/360.0);
  }

  /**
   * Converts rotations into radians for Rotation2d
   * 
   * @param rot the number of rotations to be converted
   * @return conversion of phoenix sensor values to rotation2d units
   */
  public static double rotToRadians(double rot) {
    return rot * 2.0 * Math.PI;
  }

  /**
   * Converts degrees of rotation into external encoder ticks
   * 
   * @param deg specifies degrees to be converted
   */
  public int degToExternalEncoderTicks(double deg) {
    return (int) Math.round((deg / 360.0) * EXTERNAL_ENCODER_TICKS);
  }

  /**
   * Takes in a number of ticks the internal encoder has moved and calculates the
   * number of degrees
   * the azimuth wheel rotated
   *
   * @param ticks number of ticks the internal encoder has rotated
   * @return number of degrees the wheel moved
   */
  public static double ticksToDegreesAzimuth(double ticks) {
    return (ticks / TICKS_PER_DEGREE_AZIMUTH);
  }

  /**
   * Takes in the number of degrees the wheel has/needs to rotate and calculates
   * the
   * the number of internal encoder ticks the movement equals
   *
   * @param degrees number of degrees the drive wheel has/needs to rotate
   * @return number of ticks for the drive wheel's internal encoder
   */
  public static int degreesToTicksDW(double degrees) {
    return (int) (degrees * TICKS_PER_DEGREE_DW);
  }

  /**
   * Takes in the number of ticks the internal encoder has moved and calculates
   * the number of degrees
   * the drive wheel has/needs to rotate
   *
   * @param ticks number of ticks the drive wheel has/needs to rotate
   * @return number of degrees for the movement of the drivewheel
   */
  public static double ticksToDegreesDW(double ticks) {
    return (ticks / TICKS_PER_DEGREE_DW);
  }

  /**
   * Takes in the desired degrees per second (DPS) for the drive wheel and
   * calculates the number of ticks
   * per 100 ms (units ctre wants for rate limits)
   *
   * @param degrees number of degrees per second
   * @return number of ticks per 100 ms
   */
  public static int DPSToTicksPer100msDW(double degrees) {
    return (int) (degreesToTicksDW(degrees) / 10.0);
  }

  /**
   * Takes in the desired degrees per second (DPS) for the module azimuth and
   * calculates the number of ticks
   * per 100 ms (units ctre wants for rate limits)
   *
   * @param degrees number of degrees per second
   * @return number of ticks per 100 ms
   */
  // public static int DPSToTicksPer100msAzimuth(double degrees) {
  //   return (int) (degreesToTicksAzimuth(degrees) / 10.0);
  // }

  /**
   * Converts the drive wheel's speed from ticks per 100 ms to feet per second.
   * 
   * @param rots number of rots per 100 ms
   * @return number of feet per second
   */
  public static double rotsPer100msToFPSDW(double rots) {
    return rots * 10.0 / ROTS_PER_FOOT_DW;
  }

  /**
   * Converts the drive wheel's speed from feet per second to rots per 100 ms.
   * 
   * @param feet number of feet per second
   * @return number of rots per 100 ms
   */
  public static int FPSToRotsPer100msDW(double feet) {
    return (int) (feet / 10.0 * ROTS_PER_FOOT_DW);
  }

  public static double FPStoPercentVelocity(double feet) {
    return FPSToRotsPer100msDW(feet) / DRIVE_SETPOINT_MAX;
  }

  /**
   * Returns the module heading, taking into account the gear ratio.
   *
   * @return position in azimuth encoder rotations
   */
  public double getAzimuthPosition() {
    return azimuthTalon.getPosition().getValue();
  }

  /**
   * Returns the native encoder position of the drive motor
   * 
   * @return position in relative drive encoder rotations
   */
  public double getDrivePosition() {
    return driveTalon.getPosition().getValue();
  }

  /**
   * sets the relative encoder position of the drive motor
   * 
   * @param pos position for encoder to be set to
   */
  public void setDrivePosition(double pos) {
    driveTalon.setPosition(pos);
  }

  /**
   * Returns the circumference of the module wheel in meters
   * 
   * @return circumference in meters
   */
  public static double getDriveCircumferenceMeters() {
    return DRIVE_CIRCUMFERENCE_M;
  }

  /**
   * @return speed of drive wheel in rotations per 100 ms
   */
  public double getDWSpeed() {
    return driveTalon.getVelocity().getValue();
  }

  /**
   * Get the azimuth Talon controller.
   *
   * @return azimuth Talon instance used by wheel
   */
  public TalonFX getAzimuthTalon() {
    return azimuthTalon;
  }

  /**
   * Get the drive Talon controller.
   *
   * @return drive Talon instance used by wheel
   */
  public TalonFX getDriveTalon() {
    return driveTalon;
  }

  public static double getDriveSetpointMax() {
    return DRIVE_SETPOINT_MAX;
  }

  public static double getMaxVelocityMetersPerSec() {
    return (DRIVE_SETPOINT_MAX * DRIVE_CIRCUMFERENCE_M);
  }

  public static double getAzimuthGearRatio() {
    return AZIMUTH_GEAR_RATIO;
  }

  public boolean isInverted() {
    return isInverted;
  }

  @Override
  public String toString() {
    return "Wheel{"
        + "azimuthTalon="
        + azimuthTalon
        + ", driveTalon="
        + driveTalon
        + ", DRIVE_SETPOINT_MAX="
        + DRIVE_SETPOINT_MAX
        + '}';
  }
}
