package org.team2168.subsystems;

import edu.wpi.first.apriltag.AprilTagDetector;
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
    private static NetworkTableEntry targetPoseCamera;
    private static NetworkTableEntry targetPoseRobot; 

    private static NetworkTableEntry tx;
    private static NetworkTableEntry ty;
    private static NetworkTableEntry ta;
    private static NetworkTableEntry tv;
    private static NetworkTableEntry pipeline;
    private static NetworkTableEntry camMode;

   
    AprilTagDetector detector = new AprilTagDetector();
    AprilTagDetector.Config config = new AprilTagDetector.Config();
    
    public Limelight() {

        detector.setConfig(config);
        detector.addFamily("tag32h11");
    }
    



    public enum Pipeline {
        APRIL_TAGS(0),
        PIPELINE_ONE(1),
        PIPELINE_TWO(2),
        PIPELINE_THREE(3),
        PIPELINE_FOUR(4);

        public final int pipelineValue;

        private Pipeline(int pipelineValue) {
            this.pipelineValue = pipelineValue;
        }
    }

    public enum CamMode {
        VISION_PROCESSOR(1);

        public final int camMode;

        private CamMode(int camMode) {
            this.camMode = camMode;
        }
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

  @Log(name = "Horizontal Angle: ", rowIndex = 2, columnIndex = 3)
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
        camMode.setNumber(turnOn ? 0 : 1);
        isLimelightEnabled = true;
    }

    public void enableBaseCameraSettings() {
        enableVision(true);
        camMode.setNumber(0);
        pipeline.setNumber(1);
        isLimelightEnabled = true;
    }

    public void setCamMode(int camValue) {
        camMode.setNumber(camValue);
    }

    public void setPipeline(int pipelineValue) {
        pipeline.setNumber(pipelineValue);
    }

    public void pauseLimelight() {
        setCamMode(1);
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
        targetPoseCamera = networkTable.getEntry("targetpose_cameraspace");
        targetPoseRobot = networkTable.getEntry("targetpose_robotspace");
    }



    public void periodic() {
        // This method will be called once per scheduler run\
    
        if (!isLimelightEnabled) {
          pauseLimelight();
        } else {
    
          enableVision(true);
        }
    
    }
    


    

}




