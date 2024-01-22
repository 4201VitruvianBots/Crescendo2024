// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.ModuleMap;
import java.util.List;

public class FieldSim extends SubsystemBase implements AutoCloseable {
  private final SwerveDrive m_swerveDrive;

  private final Field2d m_field2d = new Field2d();

  private Pose2d robotPose = new Pose2d(0, 0, new Rotation2d(0));
  private Pose2d intakePose;

  public FieldSim(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;

    initSim();
  }

  public void initSim() {
    SmartDashboard.putData("Field2d", m_field2d);
  }

  public Field2d getField2d() {
    return m_field2d;
  }

  public void setPath(List<Pose2d> pathPoints) {
    m_field2d.getObject("path").setPoses(pathPoints);
  }

  public void resetRobotPose(Pose2d pose) {
    m_field2d
        .getObject("Swerve Modules")
        .setPoses(ModuleMap.orderedValues(m_swerveDrive.getModulePoses(), new Pose2d[0]));
    m_field2d.setRobotPose(pose);
  }

  private void updateRobotPoses() {
    robotPose = m_swerveDrive.getPoseMeters();
    m_field2d.setRobotPose(robotPose);

//    if (RobotBase.isSimulation()) {
//      m_field2d
//          .getObject("Swerve Modules")
//          .setPoses(ModuleMap.orderedValues(m_swerveDrive.getModulePoses(), new Pose2d[0]));
//    }
  }

  @Override
  public void periodic() {
    updateRobotPoses();
  }

  @Override
  public void simulationPeriodic() {}

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
