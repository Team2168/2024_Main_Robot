// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import org.team2168.subsystems.Drivetrain;
import org.team2168.thirdcoast.swerve.Wheel;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class PoseVectorMathUtil {
    private static Pose2d robotPose;
    private static ChassisSpeeds fieldRelSpeeds;
    private static boolean useRedPositions;
    private static AprilTagFieldLayout tagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private static Pose3d targetedTagPose;

    private static double xPoseDiffMeters;
    private static double yPoseDiffMeters;
    private static double closestZeroToHeading;
    private static double rawAngleResult;

    private static final double ASSUMED_NOTE_SPEED = 3.0; // m/s
    private static double finalNoteSpeed = 0.0; // m/s

    private static double radToDeg(double radians) {
        return (radians/(Math.PI)) * 180.0;
    }

    private static double degToRad(double deg) {
        return (deg/180.0) * (Math.PI);
    }

    /**
     * Determines pose to target based on alliance color
     * 
     * @return Pose3d for target tag
     */
    private static Pose3d getTargetedTagPose() {
        if (useRedPositions) {
            return tagLayout.getTagPose(4).get();
        }
        else {
            return tagLayout.getTagPose(7).get();
        }
    }

    /**
     * getter for note speed after shooting while moving calculations
     * 
     * @return final note speed magnitude
     */
    public static double getFinalNoteSpeed() {
        return finalNoteSpeed;
    }

    /**
     * Uses Drivetrain pose (either from odometry or pose estimation)
     * to calculate angle that drivetrain should turn to
     * 
     * @param drive Drivetrain subsystem to get pose from
     * @return heading for robot to turn to (cw deg)
     */
    public static double calcHeadingToSpeaker(Drivetrain drive) {
        robotPose = drive.getPose();
        useRedPositions = (DriverStation.getAlliance().get() == Alliance.Red);

        xPoseDiffMeters = getTargetedTagPose().getX() - robotPose.getX(); // +x from blue wall to red wall
        yPoseDiffMeters = getTargetedTagPose().getY() - robotPose.getY(); // +y from source to amp

        closestZeroToHeading = drive.getHeading() - (drive.getHeading() % 360.0); // in degrees

        if (radToDeg(Math.atan(yPoseDiffMeters/xPoseDiffMeters)) < 0.0) {
            rawAngleResult = (closestZeroToHeading + (radToDeg(Math.atan(yPoseDiffMeters/xPoseDiffMeters)) - 90.0)) % 360.0; // calculation based on robot zero facing away from speaker
        }
        else {
            rawAngleResult = (closestZeroToHeading + (radToDeg(Math.atan(yPoseDiffMeters/xPoseDiffMeters)) + 90.0)) % 360.0;
        }

        return (closestZeroToHeading + rawAngleResult); // reverse sign of angle addition for ccw to cw, then subtract angle due to inverted zero
    }

    /** 
     * Fuses Drivetrain pose and ChassisSpeeds to calculate angle to turn in order
     * to target onto speaker while moving
     * 
     * @param drive Drivetrain subsystem to get pose from
     * @return heading for robot to turn to (cw deg)
     */
    public static double calcHeadingShootingWhileMoving(Drivetrain drive, double noteSpeed) {
        robotPose = drive.getPose();
        useRedPositions = (DriverStation.getAlliance().get() == Alliance.Red);
        fieldRelSpeeds = drive.getChassisSpeedFromModuleStates();

        xPoseDiffMeters = getTargetedTagPose().getX() - robotPose.getX(); // +x from blue wall to red wall
        yPoseDiffMeters = getTargetedTagPose().getY() - robotPose.getY(); // +y from source to amp

        double desiredAngle = (calcHeadingToSpeaker(drive) % 360.0); // desired resultant angle of note after kinematics calc

        double yNoteComponent = noteSpeed * Math.cos(-degToRad(desiredAngle)); // y component of note vector
        double xNoteComponent = noteSpeed * Math.sin(-degToRad(desiredAngle)); // x component of note vector

        double xChassisSpeed = fieldRelSpeeds.vxMetersPerSecond;
        double yChassisSpeed = fieldRelSpeeds.vyMetersPerSecond;

        finalNoteSpeed = Math.hypot(xNoteComponent - xChassisSpeed, yNoteComponent - yChassisSpeed); // note speed after final angle is determined

        closestZeroToHeading = drive.getHeading() - (drive.getHeading() % 360.0); // in degrees

        if (radToDeg(Math.atan((yNoteComponent - yChassisSpeed)/(xNoteComponent - xChassisSpeed))) < 0.0) {
            rawAngleResult = (closestZeroToHeading + radToDeg(Math.atan((yNoteComponent - yChassisSpeed)/(xNoteComponent - xChassisSpeed))) - 90.0) % 360.0;
        }
        else {
            rawAngleResult = (closestZeroToHeading + radToDeg(Math.atan((yNoteComponent - yChassisSpeed)/(xNoteComponent - xChassisSpeed))) + 90.0) % 360.0;
        }
        return (closestZeroToHeading + rawAngleResult);
    }

    public static double calcHeadingShootingWhileMoving(Drivetrain drive) {
        return calcHeadingShootingWhileMoving(drive, ASSUMED_NOTE_SPEED);
    }
}
