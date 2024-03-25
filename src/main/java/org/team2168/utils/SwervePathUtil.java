// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import java.util.Optional;

import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;
import org.team2168.thirdcoast.swerve.SwerveDriveConfig;
import org.team2168.thirdcoast.swerve.Wheel;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Add your docs here. */
public class SwervePathUtil {
    private static final double PATH_MAX_VEL = 5.0; // m/s // TESTING VALUE
    private static SwerveDriveConfig swerveConfig = new SwerveDriveConfig();
    private static ReplanningConfig replanningConfig = new ReplanningConfig(false, false);
    private static HolonomicPathFollowerConfig pathFollowConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.Drivetrain.kpDriveVel),
        new PIDConstants(Constants.Drivetrain.kpAngularVel, Constants.Drivetrain.kiAngularVel, Constants.Drivetrain.kdAngularVel),
        PATH_MAX_VEL, Math.hypot(swerveConfig.length, swerveConfig.width), replanningConfig);

    public static boolean getPathInvert() {
        DriverStation.refreshData();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }
        else {
            return false;
        }
    }

    public static enum InitialPathState {
        PRESERVEHEADING,
        PRESERVEODOMETRY,
        DISCARDHEADING,
    }

    public static Command getPathCommand(String pathName, Drivetrain drive, InitialPathState pathState) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (getPathInvert()) {
            path = path.flipPath();
        }

        Pose2d initialPose = path.getPreviewStartingHolonomicPose();

        SequentialCommandGroup sequence = new SequentialCommandGroup();
        
        switch(pathState) {
            case PRESERVEODOMETRY:
                break;
            case PRESERVEHEADING:
                sequence.addCommands(new InstantCommand(() -> drive.resetOdometry(initialPose, true)));
                break;
            case DISCARDHEADING:
                // drive.resetOdometry(initialPose, false);
                sequence.addCommands(new InstantCommand(() -> drive.setHeading(initialPose.getRotation().getDegrees()))); // negative to convert ccw to cw
                                                                            // setting heading to initial auto position will allow for
                                                                            // field relative swerve driving after autos finish
                sequence.addCommands(new InstantCommand(() -> drive.resetOdometry(initialPose, true)));
                break;
        }

        sequence.addCommands(followPathPlannerCommand(pathName, drive));
        return sequence;
    }

    public static Command followPathPlannerCommand(String pathName, Drivetrain drive) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return new FollowPathHolonomic(path,
        drive::getPose,
        drive::getChassisSpeeds,
        drive::driveToChassisSpeed, // TODO: verify that this will actually allow chassis to move
        SwervePathUtil.pathFollowConfig,
        () -> getPathInvert(),
        drive);
    }

    public static Command pathFindToAmp(Drivetrain drive) {
        double poseXtranslation;
        if (getPathInvert()) {
            poseXtranslation = 14.67; // red amp x position in meters
        }
        else {
            poseXtranslation = 1.85; // blue amp x position in meters
        }

        Pose2d desiredPose = new Pose2d(poseXtranslation, 7.65, new Rotation2d(Units.degreesToRadians(90.0)));
        return pathFindtoPose(drive, desiredPose);
    }

    public static Command pathFindtoPose(Drivetrain drive, Pose2d pose) {
        return new PathfindHolonomic(pose,
        new PathConstraints(PATH_MAX_VEL, PATH_MAX_VEL, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)),
        0.0,
        drive::getPose,
        drive::getChassisSpeeds,
        drive::driveToChassisSpeed,
        SwervePathUtil.pathFollowConfig,
        0.0,
        drive);
    }

    public static Command pathFindThenFollowToAmp(Drivetrain drive) {
        double poseXtranslation;
        if (getPathInvert()) {
            poseXtranslation = 14.67; // red amp x position in meters
        }
        else {
            poseXtranslation = 1.85; // blue amp x position in meters
        }

        Pose2d desiredPose = new Pose2d(poseXtranslation, 7.65, new Rotation2d(Units.degreesToRadians(90.0)));
        return pathFindToFollowPath(drive, "B_To_Amp", desiredPose);
    }

    public static Command pathFindToFollowPath(Drivetrain drive, String pathName, Pose2d pose) {
        return new PathfindThenFollowPathHolonomic(
            PathPlannerPath.fromPathFile(pathName),
            new PathConstraints(PATH_MAX_VEL, PATH_MAX_VEL, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0)),
            drive::getPose,
            drive::getChassisSpeeds,
            drive::driveToChassisSpeed,
            pathFollowConfig,
            () -> getPathInvert(),
            drive);
    }
    
    // public SwerveControllerCommand getSwerveControllerCommand(Trajectory trajectory, Drivetrain drivetrain) {
    //     return new SwerveControllerCommand(trajectory,
    //         drivetrain.getPose(),
    //         drivetrain.getKinematicsClass(),
    //         new HolonomicDriveController(
    //             new PIDController(Constants.Drivetrain.kpDriveVel, 0, 0),
    //             new PIDController(Constants.Drivetrain.kpDriveVel, 0, 0),
    //             new PIDController(Constants.Drivetrain.kpAngularVel, 0, 0),
    //             ), null, null);
    // }


    
}
