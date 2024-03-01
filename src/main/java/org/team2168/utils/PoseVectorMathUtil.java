// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2168.utils;

import org.team2168.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class PoseVectorMathUtil {
    private static Pose2d robotPose;

    public static double calcHeadingToSpeaker(Drivetrain drive) {
        robotPose = drive.getPose();
    }
}
