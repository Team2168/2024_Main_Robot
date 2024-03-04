package org.team2168.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import org.team2168.subsystems.Drivetrain;
import org.team2168.subsystems.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveWithLimelight extends CommandBase implements Loggable {

    private Drivetrain drivetrain;
    private Limelight limelight;
    private PIDController pid;
    private DoubleSupplier joystickInput;

    private static double DEFAULT_MAXANGLE = 0.0;
    private double errorToleranceAngle;
    private double limeAngle;
    private int withinThresholdLoops = 0;
    private int acceptableLoops = 10;

    private boolean manualControl;

    // TUNE THESE GAINS AND WHAT NOT

    private static final double P_NEAR = 0.05;
    private static final double P_FAR = 0.05;
    private static final double I_NEAR = 0;
    private static final double I_FAR = 0;
    private static final double MINIMUM_COMMAND = 0.2;
    private static final double MAX_INTEGRAL = 1.0;

    private double P;
    private double I;
    private double D = 0.0;

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

    @Log(name = "Turn Speed")
    private double driveLimeTurnSpeed;

    public DriveWithLimelight(Drivetrain drivetrain, Limelight limelight, double acceptableAngle, DoubleSupplier joystickInput, boolean near) {
        drivetrain = this.drivetrain;
        limelight = this.limelight;
        acceptableAngle = this.errorToleranceAngle;
        if(near) {
            P = P_NEAR;
            I = I_NEAR;
        }
        else {
            P = P_FAR;
            I = I_FAR;
        }

        manualControl = true;

        addRequirements(drivetrain);
    }

    public void initialize() {
        pid = new PIDController(P, I, D);
        limelight.enableBaseCameraSettings();

        pid.setTolerance(errorToleranceAngle);
        pid.setIntegratorRange(-MAX_INTEGRAL, MAX_INTEGRAL);
    }

    public void execute() {
        limeAngle = limelight.getOffsetX();

        if (Math.abs(limeAngle) < errorToleranceAngle) {
            ++withinThresholdLoops;
        }
        else {
            withinThresholdLoops = 0;
        }

        if (limeAngle > errorToleranceAngle) {
            driveLimeTurnSpeed = -(pid.calculate(limeAngle) + MINIMUM_COMMAND);
        }

         else if (limeAngle < errorToleranceAngle) {
            driveLimeTurnSpeed = (pid.calculate(limeAngle) - MINIMUM_COMMAND);
        }

        else {
            driveLimeTurnSpeed = 0.0;
        }

        if (withinThresholdLoops < -errorToleranceAngle) {
            drivetrain.drive(joystickInput.getAsDouble(), joystickInput.getAsDouble(), driveLimeTurnSpeed);
        }

        else if(manualControl) {
            drivetrain.drive(joystickInput.getAsDouble(), joystickInput.getAsDouble(), joystickInput.getAsDouble());
        }
    }

    


    
}
