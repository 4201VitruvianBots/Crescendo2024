// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.sim.SpawnNote;
import frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class FieldSim extends SubsystemBase implements AutoCloseable {

private Shooter m_shooter;

private ArrayList<Pose2d> m_notePoses = new ArrayList<>();
private ArrayList<Double> m_noteXvelocities = new ArrayList<>();
private ArrayList<Double> m_noteYvelocities = new ArrayList<>();

private final Field2d m_field2d = new Field2d();

private final Timer m_timer = new Timer();
private double m_lastTime = 0.0;

private Pose2d m_robotPose = new Pose2d();
private Pose2d[] m_swervePoses = {new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d()};

  public FieldSim(Shooter shooter) {
    m_timer.start();
    m_shooter = shooter;
    
    initSim();
    
    SmartDashboard.putData("SpawnNote", new SpawnNote(this));
  }

  public void initSim() {
    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void setTrajectory(Trajectory trajectory) {
    m_field2d.getObject("path").setTrajectory(trajectory);
    Logger.recordOutput("Trajectory/Auto Trajectory", trajectory);
  }

  public void spawnNote() {
    m_notePoses.add(m_robotPose);
    
    // Convert the shooter velocity to seperate x and y velocities based off the robot's heading
    double xVelocity = m_shooter.getMotorVelocityMetersPerSecond() * Math.cos(m_robotPose.getRotation().getRadians());
    double yVelocity = m_shooter.getMotorVelocityMetersPerSecond() * Math.sin(m_robotPose.getRotation().getRadians());
    
    m_noteXvelocities.add(xVelocity);
    m_noteYvelocities.add(yVelocity);
  }
  
  public void updateRobotPose(Pose2d pose) {
    m_robotPose = pose;
  }

  public void updateSwervePoses(Pose2d[] poses) {
    m_swervePoses = poses;
  }

  private void updateField2d() {
    m_field2d.setRobotPose(m_robotPose);
    m_field2d.getObject("Swerve Modules").setPoses(m_swervePoses);
    
    // Update the note poses
    for (int i = 0; i < m_notePoses.size(); i++) {
        double deltaTime = m_timer.get() - m_lastTime;
        m_lastTime = m_timer.get();

        double xVelocity = m_noteXvelocities.get(i);
        double yVelocity = m_noteYvelocities.get(i);

        double dxVelocity = xVelocity * deltaTime;
        double dyVelocity = yVelocity * deltaTime;

        Pose2d pose = m_notePoses.get(i);
        pose = pose.plus(new Transform2d(dxVelocity, dyVelocity, new Rotation2d()));
    }
    
    m_field2d.getObject("Note").setPoses(m_notePoses);

    // Remove poses that are off the field
    Iterator<Pose2d> poseIterator = m_notePoses.iterator();
    int i = 0;
    while (poseIterator.hasNext()) {
        Pose2d pose = poseIterator.next();
        if (pose.getX() > SimConstants.fieldLength || pose.getX() < 0 || pose.getY() > SimConstants.fieldWidth || pose.getY() < 0) {
            m_noteXvelocities.remove(i);
            m_noteYvelocities.remove(i);
            poseIterator.remove();
        } else {
            i++;
        }
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
