package org.team2168.subsystems;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
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
    

    public double aprilTagSize = 0.1651;
    public double fx = 2592.0;
    public double fy = 1944.0;
    public double cx = 1296.0;
    public double cy = 972.0;

    

    public int horizontalScanX = 320;
    public int horizontalScanY = 160;

    public AprilTagFieldLayout aprilTagFieldLayout;

   

    public Pose3d aprilTagInView;
    public Transform3d pose;
    public Rotation2d limelightAngle;


   
    AprilTagDetector detector = new AprilTagDetector();
    AprilTagDetector.Config config = new AprilTagDetector.Config();

    AprilTagDetection detectionSoftware;

    public enum Pipeline {
        APRIL_TAGS(0),
        SPEAKER_CENTER(1),
        PIPELINE_TWO(2),
        PIPELINE_THREE(3),
        PIPELINE_FOUR(4);

        private final int pipelineValue;

        private Pipeline(int pipelineValue) {
            this.pipelineValue = pipelineValue;
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
    return botpose.getDoubleArray(new double[6]);
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

  @Log(name = "Yaw Roation", rowIndex = 1, columnIndex = 4)
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
        botpose_wpiblue = networkTable.getEntry("botpose_wpiblue");
        
    }


 
    public void periodic() {
        // This method will be called once per scheduler run\
        Timer.getFPGATimestamp();
    
    }
    


    

}
