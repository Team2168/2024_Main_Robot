package org.team2168.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.command.Subsystem; Commented out for now, no commands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.Optional;

import org.team2168.Constants;
import org.team2168.Robot;
// import org.team2168.commands.drivetrain.DriveWithJoystick; Commented out for now, no commmands
import org.team2168.thirdcoast.swerve.*;
import org.team2168.thirdcoast.swerve.SwerveDrive.DriveMode;
import org.team2168.utils.LimelightHelpers;
import org.team2168.utils.SwervePathUtil;

public class Drivetrain extends SubsystemBase implements Loggable {
    private Wheel[] _wheels = new Wheel[SwerveDrive.getWheelCount()];
    private SwerveModulePosition[] modulePositions = new SwerveModulePosition[SwerveDrive.getWheelCount()];
    private SwerveModuleState[] moduleStates = new SwerveModuleState[SwerveDrive.getWheelCount()];
    private final boolean[] DRIVE_INVERTED = {true, false, true, false};
    private final SensorDirectionValue[] ABSOLUTE_ENCODER_INVERTED = {SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, 
        SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive};
    private final double[] ABSOLUTE_ENCODER_OFFSET = {-0.8337402, 0.2568359, -0.538818359, 0.23510742}; // the magnet offsets should be set to the opposite sign of these encoder values
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

    // pathplanner setup
    private boolean pathInvert = false;
    private static final double DEFAULT_VISION_STD_DEV = 1.0;
    private static final double PATH_MAX_VEL = 4.0; // m/s // TESTING VALUE
    private static final double PATH_MAX_MODULE_SPEED = 16.0;
    private static SwerveDriveConfig swerveConfig = new SwerveDriveConfig();
    private static ReplanningConfig replanningConfig = new ReplanningConfig(true, true);
    private static ReplanningConfig repathfindConfig = new ReplanningConfig(true, true);
    private static HolonomicPathFollowerConfig pathFollowConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.Drivetrain.kpDriveVel),
        new PIDConstants(Constants.Drivetrain.kpAngularVel, Constants.Drivetrain.kiAngularVel, Constants.Drivetrain.kdAngularVel),
        PATH_MAX_MODULE_SPEED, Math.hypot(swerveConfig.length, swerveConfig.width), replanningConfig);
    private static HolonomicPathFollowerConfig pathFindFollowConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.Drivetrain.kpDriveVel),
        new PIDConstants(Constants.Drivetrain.kpAngularVel, Constants.Drivetrain.kiAngularVel, Constants.Drivetrain.kdAngularVel),
        PATH_MAX_MODULE_SPEED, Math.hypot(swerveConfig.length, swerveConfig.width), repathfindConfig);
    private static Drivetrain instance = null;
    @Log(name = "field", width = 3, height = 2)
    private Field2d field = new Field2d(); // used to test if odometry is correct

    @Log(name = "pose estimation result", rowIndex = 3, columnIndex = 1, width = 3, height = 2)
    private Field2d poseEstfield = new Field2d();

    private SwerveDriveKinematics swerveKinematics;
    private SwerveDriveOdometry odometry;
    private ChassisSpeeds chassisSpeeds;

    public Limelight limelight = Limelight.getInstance();

    private static SwerveDrivePoseEstimator drivePoseEstimator;

    private Drivetrain() {
        // put the zeros for each module to the dashboard
        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            SmartDashboard.putNumber("Abs Zero Module " + i, Preferences.getInt(SwerveDrive.getPreferenceKeyForWheel(i), SwerveDrive.DEFAULT_ABSOLUTE_AZIMUTH_OFFSET));
        }

        //_sd.zeroAzimuthEncoders();
        _sd = configSwerve();

        SmartDashboard.putData("field", field);

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getChassisSpeeds,
            this::driveToChassisSpeed,
            pathFollowConfig,
            () -> getPathInvert(),
            this
        );
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
        driveSlot0Config.withKP(0.3);
        driveSlot0Config.withKI(0.7);
        driveSlot0Config.withKD(0.0);
        driveSlot0Config.withKV(0.0);  // TODO: tune these
        driveSlot0Config.withKA(0.0);
        driveSlot0Config.withKS(0.02);
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
            modulePositions[i] = new SwerveModulePosition(0.0, new Rotation2d(0.0));
            moduleStates[i] = new SwerveModuleState(0.0, new Rotation2d(0.0));
            SmartDashboard.putNumber("Abs position on init, module " + i, wheel.getAzimuthPosition());
        }
        SwerveDriveConfig config = new SwerveDriveConfig();
        config.wheels = _wheels;
        config.gyro = new Pigeon2(Constants.CANDevices.PIGEON_CAN_ID, "rio");
        config.gyro.setYaw(0.0);

        swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(config.lengthFromCenterToWheel, config.widthFromCenterToWheel), // Configures the position of swerve modules
            new Translation2d(config.lengthFromCenterToWheel, -config.widthFromCenterToWheel), // in order 0-3 (FL to BR)
            new Translation2d(-config.lengthFromCenterToWheel, config.widthFromCenterToWheel), // +x towards front of chassis, +y towards left of chassis
            new Translation2d(-config.lengthFromCenterToWheel, -config.widthFromCenterToWheel));

        odometry = new SwerveDriveOdometry(swerveKinematics, config.gyro.getRotation2d(), modulePositions);
        chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0.0, 0.0, 0.0, config.gyro.getRotation2d());

        drivePoseEstimator = new SwerveDrivePoseEstimator(swerveKinematics,
        config.gyro.getRotation2d(),
        modulePositions,
        getPose(),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(2.5)),
        VecBuilder.fill(1.0, 1.0, 99999999.0));

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

    public void driveWithKinematics(double forward, double strafe, double azimuth) {
        _sd.driveWithKinematics(forward, strafe, azimuth);
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(_sd.getChassisDriver(), getRotation2d());
        chassisSpeeds.omegaRadiansPerSecond = _sd.getChassisDriver().omegaRadiansPerSecond;

        moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(0.0, 0.0));

        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            _wheels[i].setWithModuleState(moduleStates[i]);
            System.out.println("desired module angle " + i + ": " + moduleStates[i]);
        }
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
     * sets gyro heading (yaw) to specified value.
     */
    public void setHeading(double angleDeg) {
        _sd.getGyro().setYaw(angleDeg);
    }

    /**
     *
     * @return the robot's heading (yaw) in degrees. Yaw positively increases with CCW rotation around Z-axis (verified from docs), so negative yaw returns CW positive.
     */
    @Log (name="Gyro Heading")
    public double getHeading() {
      return -_sd.getGyro().getYaw().getValue();
    }

    public Rotation2d getRotation2d() {
        return _sd.getGyro().getRotation2d();
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

    public void resetDriveEncoders() {
        for(int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            _wheels[i].setDrivePosition(0.0);
        }
    }

    public void resetOdometry(Pose2d pose, boolean preserveHeading) {
        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            modulePositions[i].distanceMeters = 0.0; // resets distance traveled in all module positions
        }

        resetDriveEncoders();
        if (!preserveHeading) {
            zeroGyro();
        }

        odometry.resetPosition(getRotation2d(), modulePositions, pose); // TODO: reset modulePosition distance
    }

    public void resetOdometry(Pose2d pose) {
        setHeading(pose.getRotation().getDegrees());
        resetOdometry(pose, true);
    }

    /**
     * returns the robot pose given by the SwerveDriveOdometry class
     * 
     * @return the odometry-based robot pose in meters
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public SwerveDriveKinematics getKinematicsClass() {
        return swerveKinematics;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

    /**
     * returns robot pose given by SwerveDrivePoseEstimator, infusing vision measurements with standard odometry
     * 
     * @return pose from pose estimator in meters
     */
    public Pose2d getPoseEstimate() {
        return drivePoseEstimator.getEstimatedPosition();
    }

    public void resetPoseEstimator(Pose2d pose2d, boolean preserveHeading) {
        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            modulePositions[i].distanceMeters = 0.0; // resets distance traveled in all module positions
        }

        resetDriveEncoders();
        if (!preserveHeading) {
            zeroGyro();
        }

        drivePoseEstimator.resetPosition(getRotation2d(), modulePositions, pose2d);
    }

    public double getBotposeX() {
        return drivePoseEstimator.getEstimatedPosition().getX();
    }

    public double getBotposeY() {
        return drivePoseEstimator.getEstimatedPosition().getY();
    }

    public void driveToChassisSpeed(ChassisSpeeds robotRelSpeeds) {
        chassisSpeeds = robotRelSpeeds;

        moduleStates = swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            _wheels[i].setWithModuleState(moduleStates[i]);
        }
    }

    public ChassisSpeeds getChassisSpeedFromModuleStates() {
        SwerveModuleState[] moduleStatesToConvert = new SwerveModuleState[SwerveDrive.getWheelCount()];

        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            moduleStatesToConvert[i] = new SwerveModuleState(
                _wheels[i].getDWSpeed() * Wheel.getDriveCircumferenceMeters(), // current wheel speed
                new Rotation2d(Wheel.rotToRadians(_wheels[i].getAzimuthPosition())) // current module speed
            );
        }

        return ChassisSpeeds.fromRobotRelativeSpeeds(swerveKinematics.toChassisSpeeds(moduleStatesToConvert), getRotation2d()); // TODO: verify that this actually converts to field relative
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

    public void setMotorsBrake(boolean braked) {
        if (braked) {
            for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
                _wheels[i].getAzimuthTalon().setNeutralMode(NeutralModeValue.Brake);
                _wheels[i].getDriveTalon().setNeutralMode(NeutralModeValue.Brake);
            }
        }
        else {
            for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
                _wheels[i].getAzimuthTalon().setNeutralMode(NeutralModeValue.Coast);
                _wheels[i].getDriveTalon().setNeutralMode(NeutralModeValue.Coast);
            }
        }
    }

    public void setDriveMode(SwerveDrive.DriveMode mode) {
        _sd.setDriveMode(mode);
      }
    
    public static boolean getPathInvert() {
        // DriverStation.refreshData();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
        else {
            return false;
        }
    }

    public void updatePathInvert() {
        // DriverStation.refreshData();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            pathInvert = alliance.get() == Alliance.Red;
        }
        else {
            pathInvert = false;
        }
    }

    public static enum InitialPathState {
        PRESERVEHEADING,
        PRESERVEODOMETRY,
        DISCARDHEADING,
    }

    public Command getPathCommand(String pathName, InitialPathState pathState) {
        // DriverStation.refreshData();
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (getPathInvert()) {
            path = path.flipPath();
            // path.preventFlipping = true; // prevents path from flipping multiple times if flipped once
        }
        
        Pose2d initialPose = path.getPreviewStartingHolonomicPose();

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        
        switch(pathState) {
            case PRESERVEODOMETRY:
                break;
            case PRESERVEHEADING:
                //sequence.addCommands(new SetToPose(drive, initialPose));
                sequence.addCommands(new InstantCommand(() -> resetOdometry(initialPose, true)));
                sequence.addCommands(new InstantCommand(() -> resetPoseEstimator(initialPose, true)));
                break;
            case DISCARDHEADING:
                // drive.resetOdometry(initialPose, false);
                sequence.addCommands(new InstantCommand(() -> setHeading(initialPose.getRotation().getDegrees())),
                                    new InstantCommand(() -> resetPoseEstimator(initialPose, true)),
                                    new InstantCommand(() -> resetOdometry(initialPose, true))); // negative to convert ccw to cw
                                                                            // setting heading to initial auto position will allow for
                                                                            // field relative swerve driving after autos finish
                break;
        }

        sequence.addCommands(followPathPlannerCommand(pathName));
        return sequence;
    }

    public Command followInitialPath(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

    public Command followPathPlannerCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return new FollowPathHolonomic(path,
        this::getPoseEstimate, // TODO: verify pose estimation is accurate enough for autos
        this::getChassisSpeeds,
        this::driveToChassisSpeed, // TODO: verify that this will actually allow chassis to move
        pathFollowConfig,
        () -> getPathInvert(),
        this);
    }

    // used to set up for limelight based lineup
    public Command pathFindToAmp() {
        double poseXtranslation;
        if (getPathInvert()) {
            poseXtranslation = 15.0; // red amp x position in meters
        }
        else {
            poseXtranslation = 1.5; // blue amp x position in meters
        }

        Pose2d desiredPose = new Pose2d(poseXtranslation, 6.0, new Rotation2d(Units.degreesToRadians(90.0)));
        return pathFindtoPose(desiredPose);
    }

    public Command pathFindtoPose(Pose2d pose) {
        return new PathfindHolonomic(pose,
        new PathConstraints(PATH_MAX_VEL, PATH_MAX_VEL, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)),
        0.0,
        this::getPoseEstimate,
        this::getChassisSpeeds,
        this::driveToChassisSpeed,
        pathFindFollowConfig,
        0.0,
        this);
    }

    public Command pathFindThenFollowToAmp() {
        double poseXtranslation;
        if (getPathInvert()) {
            poseXtranslation = 14.67; // red amp x position in meters
        }
        else {
            poseXtranslation = 1.85; // blue amp x position in meters
        }

        Pose2d desiredPose = new Pose2d(poseXtranslation, 8.0, new Rotation2d(Units.degreesToRadians(90.0)));
        return pathFindToFollowPath("B_To_Amp", desiredPose);
    }

    public Command pathFindThenFollowToSpeaker() {
        double poseXtranslation;
        double desiredRotation; // in degrees
        if (getPathInvert()) {
            poseXtranslation = 15.0; // red speaker position in meters
            desiredRotation = 0.0;
        }
        else {
            poseXtranslation = 1.5; // blue speaker position in meters
            desiredRotation = 180.0;
        }

        Pose2d desiredPose = new Pose2d(poseXtranslation, 5.5, new Rotation2d(Units.degreesToRadians(desiredRotation)));
        return pathFindToFollowPath("Speaker_Pathfind_End", desiredPose);
    }

    public double getDefaultShotHeading() {
        if (getPathInvert()) {
            return 0.0;
        }
        else {
            return 180.0;
        }
    }

    public enum ClimbPositions {
        BLUE_CLOSE_SPEAKER(new Pose2d(4.42, 4.91, new Rotation2d(Units.degreesToRadians(-60.0))), "B_Chain_Close_Speaker"),
        BLUE_CLOSE_SOURCE(new Pose2d(4.22, 3.29, new Rotation2d(Units.degreesToRadians(60.0))), "B_Chain_Close_Source"),
        BLUE_FAR(new Pose2d(5.83, 4.05, new Rotation2d(Units.degreesToRadians(180.0))), "B_Chain_Far"),
        RED_CLOSE_SPEAKER(new Pose2d(12.31, 4.90, new Rotation2d(Units.degreesToRadians(-120.0))), "B_Chain_Close_Speaker"),
        RED_CLOSE_SOURCE(new Pose2d(12.15, 3.27, new Rotation2d(Units.degreesToRadians(120.0))), "B_Chain_Close_Source"),
        RED_FAR(new Pose2d(10.72, 4.13, new Rotation2d(Units.degreesToRadians(0.0))), "B_Chain_Far");

        public final Pose2d desiredPose;
        public final String pathName;

        private ClimbPositions(Pose2d desiredPose, String pathName) {
            this.desiredPose = desiredPose;
            this.pathName = pathName;
        }

        public Pose2d getDesiredPose() {
            return desiredPose;
        }

        public String getPathName() {
            return pathName;
        }
    }

    public Command pathFindThenFollowToChain() {
        Pose2d currentPose = getPose();
        Pose2d desiredPose;
        double fieldCenterY = 4.1; // meters
        String pathName;

        // distinguishes pose selection based on red and blue
        if (getPathInvert()) {
            // if robot is far away, the nearest chain is the farthest chain
            if (currentPose.getX() < ClimbPositions.RED_FAR.getDesiredPose().getX()) {
                desiredPose = ClimbPositions.RED_FAR.getDesiredPose();
                pathName = ClimbPositions.RED_FAR.getPathName();
            }
            // distinguishes between last two chains through top and bottom of field
            else if (currentPose.getY() < fieldCenterY) {
                desiredPose = ClimbPositions.RED_CLOSE_SOURCE.getDesiredPose();
                pathName = ClimbPositions.RED_CLOSE_SOURCE.getPathName();
            }
            else {
                desiredPose = ClimbPositions.RED_CLOSE_SPEAKER.getDesiredPose();
                pathName = ClimbPositions.RED_CLOSE_SPEAKER.getPathName();
            }
        }
        else {
            // if robot is far away, the nearest chain is farthest
            if (currentPose.getX() > ClimbPositions.BLUE_FAR.getDesiredPose().getX()) {
                desiredPose = ClimbPositions.BLUE_FAR.getDesiredPose();
                pathName = ClimbPositions.BLUE_FAR.getPathName();
            }
            // distinguishes between last two chains through top and bottom of field
            else if (currentPose.getY() < fieldCenterY) {
                desiredPose = ClimbPositions.BLUE_CLOSE_SOURCE.getDesiredPose();
                pathName = ClimbPositions.BLUE_CLOSE_SOURCE.getPathName();
            }
            else {
                desiredPose = ClimbPositions.BLUE_CLOSE_SPEAKER.getDesiredPose();
                pathName = ClimbPositions.BLUE_CLOSE_SPEAKER.getPathName();
            }
        }

        return pathFindToFollowPath(pathName, desiredPose);
    }

    public Command pathFindToFollowPath(String pathName, Pose2d pose) {
        return new PathfindThenFollowPathHolonomic(
            PathPlannerPath.fromPathFile(pathName),
            new PathConstraints(PATH_MAX_VEL, PATH_MAX_VEL, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)),
            this::getPoseEstimate,
            this::getChassisSpeeds,
            this::driveToChassisSpeed,
            pathFindFollowConfig,
            () -> getPathInvert(),
            this);
    }

    public void visionSwervePoseEstimation() {
        LimelightHelpers.PoseEstimate visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (limelight.hasTarget() && visionPoseEstimate.tagCount >= 2) {
        drivePoseEstimator.addVisionMeasurement(visionPoseEstimate.pose,
        visionPoseEstimate.timestampSeconds,
        VecBuilder.fill(DEFAULT_VISION_STD_DEV - (visionPoseEstimate.tagCount/7.0), DEFAULT_VISION_STD_DEV - (visionPoseEstimate.tagCount/7.0), 99999999.0)); // uses Timer to account for latency
      }
    }

    @Override
    public void periodic() {
        putEncoderPositions();
        for (int i = 0; i < SwerveDrive.getWheelCount(); i++) {
            modulePositions[i] = new SwerveModulePosition(_wheels[i].getDrivenMeters(), 
            new Rotation2d(Wheel.rotToRadians(_wheels[i].getAzimuthPosition()))); // negative for counterclockwise
        }
        odometry.update(getRotation2d(), modulePositions);
        field.setRobotPose(getPoseEstimate());

        //System.out.println("chassis speed rotationSpeed: " + chassisSpeeds.omegaRadiansPerSecond);
        //System.out.println("gyro rotation2d: " + getRotation2d().getRadians());
        //System.out.println("robot Pose: " + getPose());

        drivePoseEstimator.update(getRotation2d(), modulePositions);
        // System.out.println(getPathInvert());
        visionSwervePoseEstimation();
        poseEstfield.setRobotPose(getPoseEstimate());
    }
}