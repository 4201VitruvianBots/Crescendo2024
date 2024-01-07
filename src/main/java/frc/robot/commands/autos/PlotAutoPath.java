// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.simulation.FieldSim;
import java.util.ArrayList;

public class PlotAutoPath extends Command {
  private final FieldSim m_fieldSim;
  private final ArrayList<Pose2d> m_pathPoints;
  private final String m_pathName;

  public PlotAutoPath(FieldSim fieldSim, String pathName, ArrayList<PathPlannerPath> paths) {
    var pathPoints = new ArrayList<Pose2d>();

    for (var path : paths) {
      var trajectory =
          new PathPlannerTrajectory(
              path, new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());
      pathPoints.addAll(
          trajectory.getStates().stream()
              .map(PathPlannerTrajectory.State::getTargetHolonomicPose)
              .toList());
    }

    m_fieldSim = fieldSim;
    m_pathName = pathName;
    m_pathPoints = pathPoints;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_fieldSim);
  }

  public PlotAutoPath(FieldSim fieldSim, String pathName, PathPlannerPath path) {
    var trajectory =
        new PathPlannerTrajectory(
            path, new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());
    var pathPoints =
        new ArrayList<>(
            trajectory.getStates().stream()
                .map(PathPlannerTrajectory.State::getTargetHolonomicPose)
                .toList());

    m_fieldSim = fieldSim;
    m_pathName = pathName;
    m_pathPoints = pathPoints;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_fieldSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_fieldSim.setPath(m_pathPoints);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
