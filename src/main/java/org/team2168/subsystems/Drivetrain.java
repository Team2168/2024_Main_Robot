package org.team2168.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.FilterConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.Preferences;
// import edu.wpi.first.wpilibj.command.Subsystem; Commented out for now, no commands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.beans.FeatureDescriptor;

import org.team2168.Constants;
// import org.team2168.commands.drivetrain.DriveWithJoystick; Commented out for now, no commmands
import org.team2168.thirdcoast.swerve.*;
import org.team2168.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.team2168.utils.TalonFXHelper;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Drivetrain extends SubsystemBase implements Loggable {    private Wheel[] _wheels = new Wheel[SwerveDrive.getWheelCount()];
    private final boolean[] DRIVE_INVERTED = {false, true, false, true};
    private final SensorDirectionValue[] ABSOLUTE_ENCODER_INVERTED = {SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, 
        SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive};
    private final double[] ABSOLUTE_ENCODER_OFFSET = {0.484375, 0.206299, 0.152832, 0.261230}; // the magnet offsets should be set to the opposite sign of these encoder values
    // private final double[] ABSOLUTE_ENCODER_OFFSET_DEGREES = {186.503906, 196.083984, 215.244141, 177.011719};
    private SwerveDrive _sd;
    private final boolean ENABLE_DRIVE_CURRENT_LIMIT = true;
    private final double CONTINUOUS_DRIVE_CURRENT_LIMIT = 30.0; // amps
    private final double TRIGGER_DRIVE_THRESHOLD_LIMIT = 45.0; // amps
    private final double TRIGGER_DRIVE_THRESHOLD_TIME = 0.2; // seconds

    private final boolean ENABLE_AZIMUTH_CURRENT_LIMIT = true;
    private final double CONTINUOUS_AZIMUTH_CURRENT_LIMIT = 10.0; // amps
    private final double TRIGGER_AZIMUTH_THRESHOLD_LIMIT = 10.0; // amps
    private final double TRIGGER_AZIMUTH_THRESHOLD_TIME = 0.1; // seconds

    private static Drivetrain instance = null;

    private Drivetrain() {
        // put the zeros for each module to the dashboard
        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            SmartDashboard.putNumber("Abs Zero Module " + i, Preferences.getInt(SwerveDrive.getPreferenceKeyForWheel(i), SwerveDrive.DEFAULT_ABSOLUTE_AZIMUTH_OFFSET));
        }

        //_sd.zeroAzimuthEncoders();
        _sd = configSwerve();
    }

    /**
     * @return An instance of the DriveWheel subsystem
     */
    public static Drivetrain getInstance() {
        if (instance == null)
          instance = new Drivetrain();

        return instance;
    }

    /**
     * @return a configured SwerveDrive
     */
    private SwerveDrive configSwerve() {
        Slot0Configs driveSlot0Config = new Slot0Configs();
        Slot0Configs azimuthSlot0Config = new Slot0Configs();
        MagnetSensorConfigs azimuthEncoderMagnetConfig = new MagnetSensorConfigs();

        MotorOutputConfigs driveOutputConfig = new MotorOutputConfigs();
        MotorOutputConfigs azimuthOutputConfig = new MotorOutputConfigs();

        CurrentLimitsConfigs driveCurrentConfig = new CurrentLimitsConfigs();
        CurrentLimitsConfigs azimuthCurrentConfig = new CurrentLimitsConfigs();

        FeedbackConfigs driveFeedbackConfig = new FeedbackConfigs();
        FeedbackConfigs azimuthFeedbackConfig = new FeedbackConfigs();

        MotionMagicConfigs driveMotionMagicConfig = new MotionMagicConfigs();
        MotionMagicConfigs azimuthMotionMagicConfig = new MotionMagicConfigs();

        azimuthCurrentConfig.withSupplyCurrentLimitEnable(ENABLE_AZIMUTH_CURRENT_LIMIT);
        azimuthCurrentConfig.withSupplyCurrentLimit(CONTINUOUS_AZIMUTH_CURRENT_LIMIT);
        azimuthCurrentConfig.withSupplyCurrentThreshold(TRIGGER_AZIMUTH_THRESHOLD_LIMIT);
        azimuthCurrentConfig.withSupplyTimeThreshold(TRIGGER_AZIMUTH_THRESHOLD_TIME);

        driveCurrentConfig.withSupplyCurrentLimitEnable(ENABLE_DRIVE_CURRENT_LIMIT);
        driveCurrentConfig.withSupplyCurrentLimit(CONTINUOUS_DRIVE_CURRENT_LIMIT);
        driveCurrentConfig.withSupplyCurrentThreshold(TRIGGER_DRIVE_THRESHOLD_LIMIT);
        driveCurrentConfig.withSupplyTimeThreshold(TRIGGER_DRIVE_THRESHOLD_TIME);

        azimuthFeedbackConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
        azimuthSlot0Config.withKP(40.0);
        azimuthSlot0Config.withKI(1.28);
        azimuthSlot0Config.withKD(0.0);
        azimuthSlot0Config.withKV(0.001);
        azimuthSlot0Config.withKA(0.001);
        azimuthSlot0Config.withKS(0.08);
        // azimuthSlot0Config.slot0.allowableClosedloopError = 0; // omitted from phoenix 6
        azimuthMotionMagicConfig.withMotionMagicAcceleration(150);
        azimuthMotionMagicConfig.withMotionMagicCruiseVelocity(40);
        driveFeedbackConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
        driveSlot0Config.withKP(0.5); //0.35
        driveSlot0Config.withKI(0.001);
        driveSlot0Config.withKD(0.4);
        driveSlot0Config.withKV(0.1);  // 0.032 TODO: tune these
        driveSlot0Config.withKA(0.1);
        driveSlot0Config.withKS(0.005);
        // driveSlot0Config.allowableClosedloopError = 0; // omitted from phoenix 6
        driveMotionMagicConfig.withMotionMagicAcceleration(30); // 500;
        driveMotionMagicConfig.withMotionMagicCruiseVelocity(15); // 100;

        // azimuthEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            azimuthEncoderMagnetConfig.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
            azimuthEncoderMagnetConfig.withMagnetOffset(-ABSOLUTE_ENCODER_OFFSET[i]);
            azimuthEncoderMagnetConfig.withSensorDirection(ABSOLUTE_ENCODER_INVERTED[i]);

            CANcoder azimuthEncoder = new CANcoder(Constants.CANDevices.CANCODER_ID[i], "rio");
            azimuthEncoder.getConfigurator().apply(azimuthEncoderMagnetConfig);
            azimuthEncoder.close();
            azimuthFeedbackConfig.withFeedbackRemoteSensorID(Constants.CANDevices.CANCODER_ID[i]);

            TalonFX azimuthTalon = new TalonFX(Constants.CANDevices.AZIMUTH_MODULES[i], "rio");
            azimuthTalon.getConfigurator().apply(new TalonFXConfiguration()); // sets factory default
            azimuthTalon.getConfigurator().apply(azimuthSlot0Config);
            azimuthTalon.getConfigurator().apply(azimuthFeedbackConfig);
            azimuthTalon.getConfigurator().apply(azimuthMotionMagicConfig);
            azimuthTalon.getConfigurator().apply(azimuthOutputConfig);
            azimuthTalon.getConfigurator().apply(azimuthCurrentConfig);
            azimuthTalon.setInverted(true);
            azimuthTalon.setNeutralMode(NeutralModeValue.Brake);
            // System.out.println("configured azimuth motor: " + i);

            TalonFX driveTalon = new TalonFX(Constants.CANDevices.DRIVE_MOTORS[i], "rio");
            driveTalon.getConfigurator().apply(new TalonFXConfiguration());
            driveTalon.getConfigurator().apply(driveSlot0Config);
            driveTalon.getConfigurator().apply(driveFeedbackConfig);
            driveTalon.getConfigurator().apply(driveMotionMagicConfig);
            driveTalon.getConfigurator().apply(driveOutputConfig);
            driveTalon.getConfigurator().apply(driveCurrentConfig);
            driveTalon.setInverted(DRIVE_INVERTED[i]);
            // System.out.println("configured drive motor: " + i);
            driveTalon.setNeutralMode(NeutralModeValue.Brake);

            Wheel wheel = new Wheel(azimuthTalon, driveTalon);
            _wheels[i] = wheel;

            // set the value of the internal encoder's current position to that of the external encoder,
            // taking into account the gear ratio & difference in resolution, as well as the saved zero
            // Preferences prefs = Preferences.getInstance();
            // _wheels[i].setAzimuthZero(Preferences.getInt(SwerveDrive.getPreferenceKeyForWheel(i), SwerveDrive.DEFAULT_ABSOLUTE_AZIMUTH_OFFSET));
            _wheels[i].setAzimuthZero(ABSOLUTE_ENCODER_OFFSET[i]);
            _wheels[i].setDriveMode(DriveMode.OPEN_LOOP);
            SmartDashboard.putNumber("Abs position on init, module " + i, wheel.getAzimuthPosition());
        }

        SwerveDriveConfig config = new SwerveDriveConfig();
        config.wheels = _wheels;
        config.gyro = new PigeonIMU(Constants.CANDevices.PIGEON_IMU_CAN_ID);
        config.gyro.setYaw(0.0);
        return new SwerveDrive(config);
    }

    /**
     * Drive the robot in given field-relative direction and with given rotation.
     *
     * @param forward Y-axis movement, from -1.0 (reverse) to 1.0 (forward)
     * @param strafe X-axis movement, from -1.0 (left) to 1.0 (right)
     * @param azimuth robot rotation, from -1.0 (CCW) to 1.0 (CW)
     */
    public void drive(double forward, double strafe, double azimuth) {
        _sd.drive(forward, strafe, azimuth);
    }

    public Wheel[] getWheels() {
        return _wheels;
    }

    /**
     * sets the gyros heading (yaw) to 0 degrees.
     */
    public void zeroGyro() {
      _sd.getGyro().setYaw(0.0);
    }

    /**
     *
     * @return the robot's heading (yaw) in degrees. Yaw positively increases in the CW direction TODO: verify this is accurate
     */
    @Log (name="Gyro Heading")
    public double getHeading() {
      double ypr_deg[] = new double[3];
      _sd.getGyro().getYawPitchRoll(ypr_deg);
      return -ypr_deg[0];
    }

    /**
     * Set the absolute module heading in terms of the module
     *
     * @param position position in motor ticks
     */
    public void setAzimuth(Wheel wheel, int position) {
        wheel.setAzimuthPosition(position);
    }

    /**
     * Puts azimuth positions in degrees to the SmartDashboard
     */
    public void putAzimuthPositions() {
        Wheel[] wheels = _sd.getWheels();
        for(int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            SmartDashboard.putNumber("Azimuth angle " + i, Wheel.ticksToDegreesAzimuth(wheels[i].getAzimuthPosition()));
        }
    }

    /**
     * Puts external and calculated internal encoder positions to the SmartDashboard
     */
    public void putEncoderPositions() {
        Wheel[] wheels = _sd.getWheels();
        for(int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            SmartDashboard.putNumber("External encoder pos " + i, wheels[i].getAzimuthPosition());
        }
    }


    /**
     * Save the wheels' azimuth current position as read by absolute encoder. These values are saved
     * persistently on the roboRIO and are normally used to calculate the relative encoder offset
     * during wheel initialization.
     *
     * <p>The wheel alignment data is saved in the WPI preferences data store and may be viewed using
     * a network tables viewer.
     *
     * @see #zeroAzimuthEncoders()
     */
    public void saveAzimuthPositions() {
        _sd.saveAzimuthPositions();
    }

    /**
     * Stops all wheels' azimuth and drive movement. Calling this in the robots {@code teleopInit} and
     * {@code autonomousInit} will reset wheel azimuth relative encoders to the current position and
     * thereby prevent wheel rotation if the wheels were moved manually while the robot was disabled.
     */
    public void stop() {
        _sd.stop();
    }


    public void setDriveMode(SwerveDrive.DriveMode mode) {
        _sd.setDriveMode(mode);
      }

    @Override
    public void periodic() {
        putEncoderPositions();
    }
}

