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
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Add your docs here. */
public class SwervePathUtil {
    private static final double PATH_MAX_VEL = 3.0; // m/s // TESTING VALUE
    private static SwerveDriveConfig swerveConfig = new SwerveDriveConfig();
    private static ReplanningConfig replanningConfig = new ReplanningConfig();
    private static HolonomicPathFollowerConfig pathFollowConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.Drivetrain.kpDriveVel),
        new PIDConstants(Constants.Drivetrain.kpAngularVel),
        PATH_MAX_VEL, Math.hypot(swerveConfig.length, swerveConfig.width), replanningConfig);
    public static boolean getPathInvert() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.get() == Alliance.Red;
    }

    public static enum InitialPathState {
        PRESERVEHEADING,
        PRESERVEODOMETRY,
        DISCARDHEADING,
    }

    public static Command getPathCommand(String pathName, Drivetrain drive, InitialPathState pathState) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
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
