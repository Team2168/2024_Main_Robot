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
    private PIDController pid;

    public double targetDistanceX;
    public double targetDistanceZ;
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

    private double drivePoseEst;
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
            targetDistanceZ = limelight.getTargetPoseZ() - limelight.getBotPoseZ();
        }
        else if(ally.get() == Alliance.Blue) {
            targetDistanceX = limelight.getBotPoseX() + limelight.getTargetPoseX();
            targetDistanceZ = limelight.getBotPoseZ() + limelight.getTargetPoseZ();
        }

        targetAngle = limelight.getTargetPoseYaw();

        pid = new PIDController(P, I, D);
        limelight.enableBaseCameraSettings();
        limelight.setPipeline(1);

        pid.setTolerance(minimumError);
        pid.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);
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

        if (targetDistanceX >= minimumError && targetDistanceZ >= minimumError && targetAngle >= 5.0) {
            drivePoseEst = (pid.calculate((targetDistanceX + targetDistanceZ + targetAngle) / 3));
        }

        // halves speed once closer to target
        else if (targetDistanceX >= (minimumError / 2) && targetDistanceZ >= (minimumError / 2) && targetAngle >= 0.0) {
            drivePoseEst = (pid.calculate((targetDistanceX + targetDistanceZ + targetAngle) / 6));
        }

        // loses all speed once in area needed
        else {
            drivePoseEst = 0.0;
        }

        if (withinThresholdLoops < acceptableLoops) {
            drivetrain.drive(oi.getDriverJoystickYValue(), oi.getDriverJoystickXValue(), drivePoseEst);
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
