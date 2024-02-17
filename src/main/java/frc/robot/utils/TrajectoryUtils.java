// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;

public class TrajectoryUtils {

  public static FollowPathHolonomic generatePPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive, String pathName, double maxSpeed) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, false);
  }

  public static FollowPathHolonomic generatePPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive, String pathName, double maxSpeed, boolean manualFlip) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, manualFlip);
  }

  public static FollowPathHolonomic generatePPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive, PathPlannerPath path, double maxSpeed) {

    return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, false);
  }

  public static FollowPathHolonomic generatePPHolonomicCommand(
      CommandSwerveDrivetrain swerveDrive,
      PathPlannerPath path,
      double maxSpeed,
      boolean manualFlip) {
    return new FollowPathHolonomic(
        path,
        () -> swerveDrive.getState().Pose,
        swerveDrive::getChassisSpeed,
        swerveDrive::setChassisSpeedControl,
        new HolonomicPathFollowerConfig(
            new PIDConstants(DRIVE.kP_X, DRIVE.kI_X, DRIVE.kD_X),
            new PIDConstants(DRIVE.kP_Theta, DRIVE.kI_Theta, DRIVE.kD_Theta),
            maxSpeed,
            0.86210458762,
            new ReplanningConfig(false, false, 1.0, 0.25)),
        () -> manualFlip && Controls.isRedAlliance(),
        swerveDrive);
  }

  public static Command generateChoreoCommand(
      CommandSwerveDrivetrain swerveDrive, String pathName, double maxSpeed, boolean flipPath) {
    ChoreoTrajectory traj = Choreo.getTrajectory(pathName);

    return generateChoreoCommand(swerveDrive, traj, maxSpeed, flipPath);
  }

  public static Command generateChoreoCommand(
      CommandSwerveDrivetrain swerveDrive,
      ChoreoTrajectory traj,
      double maxSpeed,
      boolean flipPath) {
    return Choreo.choreoSwerveCommand(
        traj,
        () -> swerveDrive.getState().Pose,
        new PIDController(DRIVE.kP_X, DRIVE.kI_X, DRIVE.kD_X),
        new PIDController(DRIVE.kP_X, DRIVE.kI_X, DRIVE.kD_X),
        new PIDController(DRIVE.kP_Theta, DRIVE.kI_Theta, DRIVE.kD_Theta),
        swerveDrive::setChassisSpeedControl,
        () -> flipPath,
        swerveDrive);
  }
}
