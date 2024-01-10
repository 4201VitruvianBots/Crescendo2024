// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.subsystems.SwerveDrive;

public class TrajectoryUtils {
  public static FollowPathHolonomic generatePPHolonomicCommand(
      SwerveDrive swerveDrive, String pathName, double maxSpeed, boolean flipPath) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, flipPath);
  }

  public static FollowPathHolonomic generatePPHolonomicCommand(
      SwerveDrive swerveDrive, PathPlannerPath path, double maxSpeed, boolean flipPath) {
    return new FollowPathHolonomic(
        path,
        swerveDrive::getPoseMeters,
        swerveDrive::getChassisSpeeds,
        swerveDrive::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(DRIVE.kP_X, DRIVE.kI_X, DRIVE.kD_X),
            new PIDConstants(DRIVE.kP_Theta, DRIVE.kI_Theta, DRIVE.kD_Theta),
            maxSpeed,
            0.86210458762,
            new ReplanningConfig(false, false, 1.0, 0.25)),
        () -> flipPath,
        swerveDrive);
  }
}
