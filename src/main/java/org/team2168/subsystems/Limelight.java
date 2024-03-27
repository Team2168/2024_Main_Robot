package org.team2168.subsystems;


import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;


import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log; 

public class Limelight extends SubsystemBase implements Loggable {
    private static Limelight instance = null;
    private NetworkTable networkTable;

    private static boolean isLimelightEnabled;
    private static NetworkTableEntry getPipeline;
    private static NetworkTableEntry pipeline;
    private static NetworkTableEntry camMode;
  
    private static NetworkTableEntry tx;
    private static NetworkTableEntry ty;
    private static NetworkTableEntry ta;
    private static NetworkTableEntry tv;
    private static NetworkTableEntry tl;
    private static NetworkTableEntry cl;
    private static NetworkTableEntry botpose;
    private static NetworkTableEntry botpose_wpiblue;
    private static NetworkTableEntry targetpose;
    

    public double aprilTagSize = 0.1651;
    public double fx = 2592.0;
    public double fy = 1944.0;
    public double cx = 1296.0;
    public double cy = 972.0;

    

    

    public int horizontalScanX = 320;
    public int horizontalScanY = 160;

    public AprilTagFieldLayout aprilTagFieldLayout;

    
    double heightOffset;
    double heightToLimelight = 0.648; // m
    double heightToSpeakerTag = 1.448; // m
    double distanceFromTarget = 0.0;
    
    double limelightAngleDegrees = 29.45;

   

    public Pose3d aprilTagInView;
    public Transform3d pose;
    public Rotation2d limelightAngle;

    


   
    AprilTagDetector detector = new AprilTagDetector();
    AprilTagDetector.Config config = new AprilTagDetector.Config();

    AprilTagDetection detectionSoftware;

    public enum Pipeline {
        ALL_APRIL_TAGS(0),
        SPEAKERS(1),
        HUMAN_PLAYER_STATIONS(2),
        HIGHRES_LOWFPS(3),
        AMPS(4),
        STAGES(5);

        public final int pipelineValue;

        private Pipeline(int pipelineValue) {
            this.pipelineValue = pipelineValue;
        }

        public int getPipeline() {
          return pipelineValue;
        }
    }

    public Limelight() {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        init();
        isLimelightEnabled = false;
    }

    
    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    


    @Log(name = "isLimelightOn?", rowIndex = 2, columnIndex = 5)
    public boolean checkLimelightPower() {
        return isLimelightEnabled();
    }

    @Log(name = "hasTarget:", rowIndex = 2, columnIndex = 4)
  public boolean hasTarget() {
    return tv.getDouble(0.0) == 1.0;
  }


  @Log(name = "X offset", rowIndex = 2, columnIndex = 3)
  public double getOffsetX() {
    return tx.getDouble(0.0);
  }
 
  @Log(name ="Y offset", rowIndex = 3, columnIndex = 3)
  public double getOffsetY() {
    return ty.getDouble(0.0);
  }

  
  public double[] getBotPoseArray() {
    return botpose_wpiblue.getDoubleArray(new double[6]); // uses blue origin to match odometry positions
  }

  public double[] getTargetPoseArray() {
    return targetpose.getDoubleArray(new double[6]);
  }

  @Log(name = "X pose", rowIndex = 1, columnIndex = 1)
  public double getBotPoseX() {
    return getBotPoseArray()[0];
  }

  @Log(name = "Y pose", rowIndex = 1, columnIndex = 2)
  public double getBotPoseY() {
    return getBotPoseArray()[1];
  }

  @Log(name = "Z pose", rowIndex = 1, columnIndex = 3)
  public double getBotPoseZ() {
    return getBotPoseArray()[2];
  }

   @Log(name = "target X pose", rowIndex = 2, columnIndex = 1)
  public double getTargetPoseX() {
    return getTargetPoseArray()[0];
  }

  @Log(name = "target Y pose", rowIndex = 2, columnIndex = 2)
  public double getTargetPoseY() {
    return getTargetPoseArray()[1];
  }

  @Log(name = "target Z pose", rowIndex = 2, columnIndex = 3)
  public double getTargetPoseZ() {
    return getTargetPoseArray()[2];
  }

  @Log(name = "target Yaw rotation", rowIndex = 2, columnIndex = 4)
  public double getTargetPoseYaw() {
    return getTargetPoseArray()[5];
  }


  @Log(name = "bot Yaw Roation", rowIndex = 1, columnIndex = 4)
  public double getBotRotationYaw() {
    return getBotPoseArray()[5];
  }

    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    public void enableVision(boolean turnOn) {
        setCamMode(1);
        isLimelightEnabled = true;
    }

    public void enableBaseCameraSettings() {
        enableVision(true);
        setCamMode(0);
        setPipeline(1);
        isLimelightEnabled = true;
    }

    public void setCamMode(int camValue) {
        camMode.setNumber(camValue);
    }

    public void setPipeline(int pipelineValue) {
        pipeline.setNumber(pipelineValue);
    }

    public int getPipeline() {
      return getPipeline.getNumber(0.0).intValue();
    }

    public void pauseLimelight() {
        setCamMode(0);
        setPipeline(0);
        isLimelightEnabled = false;
    }

    public Rotation2d getRotation2d() {
      return Rotation2d.fromDegrees(getBotPoseArray()[5]);
    }

    public Pose2d getPose2d() {
      return new Pose2d(getBotPoseX(), getBotPoseY(), getRotation2d());
    }

    @Log(name = "distance calc (limelight)", rowIndex = 0, columnIndex = 0)
    public double calculateDistance() {
      double currentPipeline = getPipeline();

      if (currentPipeline == 1) {
          heightOffset = heightToSpeakerTag - heightToLimelight;
      }
      else if (currentPipeline == 2) {
          heightOffset = 0.0;
      }
      
      distanceFromTarget = (heightOffset)/Math.tan(Units.degreesToRadians(limelightAngleDegrees + getOffsetY()));

      return distanceFromTarget;
  }

  public double getLimelightLatencySec() {
    return cl.getDouble(0.0)/1000.0; // converts ms latency into seconds
  }
    
    public boolean isLimelightEnabled() {
        return isLimelightEnabled;
    }

    public void init() {
        tv = networkTable.getEntry("tv");
        tx = networkTable.getEntry("tx");
        ty = networkTable.getEntry("ty");
        ta = networkTable.getEntry("ta");
        tl = networkTable.getEntry("tl");
        cl = networkTable.getEntry("cl");
        getPipeline = networkTable.getEntry("getpipe");
        camMode = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");
        botpose = networkTable.getEntry("botpose");
        targetpose = networkTable.getEntry("targetpose");
        botpose_wpiblue = networkTable.getEntry("botpose_wpiblue");
        
    }


 
    public void periodic() {
        // This method will be called once per scheduler run\
        Timer.getFPGATimestamp();
        //System.out.println(calculateDistance());
    
    }
    


    

}
