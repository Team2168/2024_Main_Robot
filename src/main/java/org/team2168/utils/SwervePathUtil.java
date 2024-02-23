// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import org.team2168.Constants;
import org.team2168.subsystems.Drivetrain;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** Add your docs here. */
public class SwervePathUtil {

    public static enum InitialPathState {
        PRESERVEHEADING,
        PRESERVEODOMETRY,
        DISCARDHEADING,
    }

    public Command followPathPlannerCommand(String pathName, Drivetrain drive) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return new FollowPathHolonomic(drive.getPose(),
        drive.getChassisSpeeds(),
        Chassi
        )
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
