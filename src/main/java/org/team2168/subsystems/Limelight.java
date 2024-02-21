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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

   
    AprilTagDetector detector = new AprilTagDetector();
    AprilTagDetector.Config config = new AprilTagDetector.Config();

    AprilTagDetection detectionSoftware;

    public enum Pipeline {
        APRIL_TAGS(0),
        PIPELINE_ONE(1),
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

        CameraServer.startAutomaticCapture();
        CvSink cvSink = CameraServer.getVideo();

        detector.setConfig(config);
        detector.addFamily("tag16h5");

        AprilTagPoseEstimator.Config poseEstConfig = new AprilTagPoseEstimator.Config(aprilTagSize, cx, cy, fx, fy);
        AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(poseEstConfig);

        Mat mat = new Mat();
        Mat grayMat = new Mat();

        while (!Thread.interrupted()) {
            // grab image from camera
            long time = cvSink.grabFrame(mat);
            if (time == 0) {
              continue;  // error getting image
            }
           
            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_BGR2GRAY);

             // run detection
            for (AprilTagDetection detection : detector.detect(grayMat)) {
            // filter by property

                // run pose estimator
                 pose = estimator.estimate(detection);
             }

             
        }

        

            

    }

    @Log(name = "X location", rowIndex = 4, columnIndex = 1)
    public double getLocationX() {
        return pose.getX();
     }

    @Log(name = "Y location", rowIndex = 4, columnIndex = 1)
    public double getLocationY() {
        return pose.getY();
     }

     @Log(name = "Z location", rowIndex = 4, columnIndex = 1)
    public double getLocationZ() {
        return pose.getZ();
     }



    

    


    



    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    @Config(name = "hasTarget:")
  public boolean hasTarget() {
    return tv.getDouble(0.0) == 1.0;
  }

  @Log(name = "Horizontal Angle", rowIndex = 2, columnIndex = 3)
  public double getOffsetX() {
    return tx.getDouble(0.0);
  }
 
  @Log(name = "Vertical Angle", rowIndex = 3, columnIndex = 3)
  public double getOffsetY() {
    return ty.getDouble(0.0);
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


    public boolean isLimelightEnabled() {
        return isLimelightEnabled;
    }


    

    


    public void init() {
        tv = networkTable.getEntry("tv");
        tx = networkTable.getEntry("tx");
        ty = networkTable.getEntry("ty");
        ta = networkTable.getEntry("ta");
        getPipeline = networkTable.getEntry("getpipe");
        camMode = networkTable.getEntry("camMode");
        pipeline = networkTable.getEntry("pipeline");

        
    }


 
    public void periodic() {
        // This method will be called once per scheduler run\
    
    }
    


    

}
