package org.team2168.commands.Drivetrain;

import java.util.Optional;

import org.team2168.OI;
import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveWithPoseEst extends CommandBase implements Loggable {

    private Drivetrain drivetrain;
    private Limelight limelight;
    private OI oi;
    private PIDController pidx;
    private PIDController pidy;
    private PIDController pidAngle;

    public double targetDistanceX;
    public double targetDistanceY;
    public double targetAngle;
    public double chassisRot;

    public double minimumError = 10.0;
    private double kDriveInvert = 1.0;
    private double MAX_INTEGRAL = 1.0;
    private int acceptableLoops = 10;
    private int withinThresholdLoops = 0;

    private static final double P_NEAR = 0.01;
    private static final double P_FAR = 0.01;
    private static final double I_NEAR = 0;
    private static final double I_FAR = 0;

    private double P;
    private double I;
    private double D = 0.003;

    @Config
    void setLimeP(double P) {
        this.P = P;
    }

    @Config 
    void setLimeI(double I) {
        this.I = I;
    }

    @Config
    void setLimeD(double D) {
        this.D = D;
    }

    private double drivePoseEstX;
    private double drivePoseEstY;
    private double driveTargetAngle;
    private boolean manualControl;

    Optional<Alliance> ally = DriverStation.getAlliance();
    
    
    public DriveWithPoseEst(Drivetrain drivetrain, Limelight limelight, double minimumError, boolean near) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.minimumError = minimumError;

        if(near) {
            P = P_NEAR;
            I = I_NEAR;
        }
        else {
            P = P_FAR;
            I = I_FAR;
        }

        addRequirements(drivetrain, limelight);
    }

    
    public void initialize() {
        
        if(ally.get() == Alliance.Red) {
            targetDistanceX = limelight.getTargetPoseX() - limelight.getBotPoseX();
            targetDistanceY = limelight.getTargetPoseY() - limelight.getBotPoseY();
        }
        else if(ally.get() == Alliance.Blue) {
            targetDistanceX = limelight.getBotPoseX() + limelight.getTargetPoseX();
            targetDistanceY = limelight.getBotPoseY() + limelight.getTargetPoseY();
        }

        targetAngle = limelight.getTargetPoseYaw();

        pidx = new PIDController(P, I, D);
        pidy = new PIDController(P, I, D);
        pidAngle = new PIDController(P, I, D);
        limelight.enableBaseCameraSettings();
        limelight.setPipeline(1);

        pidx.setTolerance(minimumError);
        pidx.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);

        pidy.setTolerance(minimumError);
        pidy.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);

        pidAngle.setTolerance(minimumError);
        pidAngle.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);
        oi = OI.getInstance();

        if (DriverStation.getAlliance().get() == Alliance.Red) {
          kDriveInvert = -1.0;
        }
    }

    
    public void execute() {
        if (limelight.hasTarget()) {
            manualControl = false;
        }
        else {
            manualControl = true;
        }

        if (Math.abs(targetAngle) < 10.0) {
            ++withinThresholdLoops;
        }
        else {
            withinThresholdLoops = 0;
        }

        if (targetDistanceX >= minimumError) {
            drivePoseEstX = (pidx.calculate(targetDistanceX));
        }

        // halves speed once closer to target
        else if (targetDistanceX >= (minimumError / 2)) {
            drivePoseEstX = (pidx.calculate((targetDistanceX / 2)));
        }

        // loses all speed once in area needed
        else {
            drivePoseEstX = 0.0;
        }

        if (targetDistanceY >= minimumError) {
            drivePoseEstY = (pidy.calculate(targetDistanceY));
        }

        // halves speed once closer to target
        else if (targetDistanceY >= (minimumError / 2)) {
            drivePoseEstY = (pidy.calculate((targetDistanceY / 2)));
        }

        // loses all speed once in area needed
        else {
            drivePoseEstY = 0.0;
        }

        if (driveTargetAngle >= 5.0) {
            driveTargetAngle = (pidAngle.calculate(driveTargetAngle));
        }

        else if (driveTargetAngle >= 0.0) {
            driveTargetAngle = (pidAngle.calculate(driveTargetAngle));
        }

        else {
            driveTargetAngle = 0.0;
        }

        if (withinThresholdLoops < acceptableLoops) {
            drivetrain.drive(drivePoseEstX, drivePoseEstY, driveTargetAngle);
        }

        else if(manualControl) {
            if (oi.driverJoystick.isPressedButtonA()) {
                chassisRot = 0.35;
            }
            else if (oi.driverJoystick.isPressedButtonB()) {
                chassisRot = -0.35;
            }
            else {
                chassisRot = 0.0;
            }
            drivetrain.drive(oi.getDriverJoystickYValue() * kDriveInvert, oi.getDriverJoystickXValue() * kDriveInvert, chassisRot);
            
        }
        
    }
    
    public void end(boolean interrupted) {
        if (manualControl) {
            limelight.pauseLimelight();
        }
    }

    public boolean isFinished() {
        return false;
    }


}
