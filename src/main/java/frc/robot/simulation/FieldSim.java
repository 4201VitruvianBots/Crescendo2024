// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;

public class FieldSim extends SubsystemBase implements AutoCloseable {

  private final Field2d m_field2d = new Field2d();

  private Pose2d m_robotPose = new Pose2d();
  private Pose2d[] m_swervePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};

  public FieldSim() {
    initSim();
  }

  public void initSim() {
    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void setPath(List<Pose2d> pathPoints) {
    m_field2d.getObject("path").setPoses(pathPoints);
  }

  public void updateRobotPose(Pose2d pose) {
    m_robotPose = pose;
  }

  public void updateSwervePoses(Pose2d[] poses) {
    m_swervePoses = poses;
  }

  private void updateField2d() {
    m_field2d.setRobotPose(m_robotPose);

    if (RobotBase.isSimulation()) {
      m_field2d.getObject("Swerve Modules").setPoses(m_swervePoses);
    }
  }

  @Override
  public void periodic() {
    updateField2d();
  }

  @Override
  public void simulationPeriodic() {}

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
