package org.team2168.thirdcoast.motion;

import java.io.File;

import org.team2168.subsystems.Drivetrain;
import org.team2168.thirdcoast.swerve.SwerveDrive;
import org.team2168.thirdcoast.swerve.Wheel;
import org.team2168.thirdcoast.util.Setpoint;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import io.github.oblarg.oblog.annotations.Log;

public class PathController implements Runnable {

  private static final int NUM_WHEELS = 4;
  private static final double ROTS_PER_FOOT = Wheel.ROTS_PER_FOOT_DW;
  private static final Drivetrain DRIVE = Drivetrain.getInstance();

  @SuppressWarnings("FieldCanBeLocal")
  private static final double yawKp = 0.01; // 0.03

  private static final double percentToDone = 0.50;
  // timestep
  private static final double DT = 0.02;

  private static final double MIN_VEL = 0.0; // 0.07 x max motor output
  //private static final double MIN_START = 0.2; // 0.07 x max motor output
  private static final double MIN_START = 1.0/12.0; // ft/s
  //  private static final double RATE_CAP = 0.35;
  //  private static final RateLimit rateLimit = new RateLimit(0.015);
  private final int PID = 0;
  private Trajectory trajectory;
  private Notifier notifier;
  private Wheel[] wheels;
  private States state;
  private double maxVelocityFtSec;
  private double yawDelta;
  private int iteration;
  private double[] start;
  private Setpoint setpoint;
  private double setpointPos;
  private double yawError;
  private boolean isDriftOut;

  private double yaw;
  private double forward;
  private double strafe;

 /**
  * 
  * @param pathName
  * @param yawDelta in degrees, positive being clockwise
  * @param isDriftOut
  */
  public PathController(String pathName, double yawDelta, boolean isDriftOut) {
    this.yawDelta = yawDelta;
    this.isDriftOut = isDriftOut;
    wheels = DRIVE.getWheels();
    File csvFile = new File(Filesystem.getDeployDirectory().getPath() + "/paths/output/" + pathName + ".pf1.csv");

    trajectory = new Trajectory(csvFile);
  }

  public void start() {
    start = new double[4];
    notifier = new Notifier(this);
    notifier.startPeriodic(DT);
    state = States.STARTING;
  }

  public boolean isFinished() {
    return state == States.STOPPED;
  }

  @Override
  public void run() {

    switch (state) {
      case STARTING:
        logState();
        double rotsPerSecMax = Wheel.getDriveSetpointMax() * 10.0;
        maxVelocityFtSec = rotsPerSecMax / ROTS_PER_FOOT; //~15.8 ft/s
        iteration = 0;
        DRIVE.setDriveMode(SwerveDrive.DriveMode.CLOSED_LOOP);

        for (int i = 0; i < NUM_WHEELS; i++) {
          start[i] = wheels[i].getDriveTalon().getPosition().getValue();
        }

        double currentAngle = DRIVE.getHeading();
        setpoint = new Setpoint(currentAngle, yawDelta, percentToDone);

        logInit();
        state = States.RUNNING;
        break;
      case RUNNING:
        if (iteration == trajectory.length() - 1) {
          state = States.STOPPING;
        }

        Trajectory.Segment segment = trajectory.getIteration(iteration);

        double currentProgress = iteration / (double) trajectory.length();

        double segmentVelocity = segment.velocity;
        if (segment.velocity < MIN_START) {
          segmentVelocity = MIN_START;
        }

        double setpointVelocity = segmentVelocity / maxVelocityFtSec;

        forward = Math.cos(segment.heading) * setpointVelocity;
        strafe = Math.sin(segment.heading) * setpointVelocity;

        if (currentProgress > percentToDone && segment.velocity < MIN_VEL) {
          state = States.STOPPING;
        }

        setpointPos = setpoint.getSetpoint(currentProgress);

        yawError = setpointPos - DRIVE.getHeading();
        //double yaw;

        yaw = yawError * yawKp;

        if (forward > 1d || strafe > 1d) System.out.printf("forward = {} strafe = {}", forward, strafe);

        DRIVE.drive(forward, strafe, yaw);
        System.out.println(iteration + "," + forward + "," + strafe + "," + yaw + "," + segmentVelocity + "," + setpointVelocity);
        SmartDashboard.putNumber("Auto commanded fwd speed normalized", forward);
        SmartDashboard.putNumber("Auto commanded fwd speed FPS", forward * maxVelocityFtSec);
        
        iteration++;
        break;
      case STOPPING:
        DRIVE.setDriveMode(SwerveDrive.DriveMode.OPEN_LOOP);
        logState();
        state = States.STOPPED;
        break;
      case STOPPED:
        logState();
        notifier.close();
        break;
    }
  }

  @Log private void logState() {
    System.out.printf("{}", state);
  }

  private void logInit() {
    System.out.printf(
        "Path start yawKp = {} yawDelta = {} maxVelocity in/s = {}",
        yawKp,
        yawDelta,
        maxVelocityFtSec);
  }

  public double getYawError() {
    return yawError;
  }

  public double getSetpointPos() {
    return setpointPos;
  }

  public void interrupt() {
    System.out.println("interrupted");
    state = States.STOPPED;
  }

  @Log(name = "PathController calculated fwd normalized")
  private double getFwdNormalized() {
    return forward;
  }

  @Log(name = "PathController calculated fwd FPS")
  private double getFwdFPS()  {
    return forward * maxVelocityFtSec;
  }

  @Log(name = "PathController calculated strafe normalized")
  private double getStrafeNormalized() {
    return strafe;
  }

  @Log(name = "PathController calculated strafe FPS")
  private double getStrafeFPS() {
    return strafe;
  }

  @Log(name = "PathController calculated yaw normalized")
  private double getYawNormalized() {
    return yaw;
  }

  @Log(name = "PathController calculated yaw degrees")
  private double getYawDegrees() {
    return yaw * 360;
  }
}