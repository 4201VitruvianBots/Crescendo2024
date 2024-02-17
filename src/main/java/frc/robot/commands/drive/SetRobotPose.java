// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import java.util.function.BooleanSupplier;

public class SetRobotPose extends Command {
  /** Creates a new RestGyro. */
  CommandSwerveDrivetrain m_swerveDrive;

  Pose2d m_pose;
  BooleanSupplier m_booleanSupplier;

  public SetRobotPose(CommandSwerveDrivetrain swerveDrive, Pose2d pose) {
    this(swerveDrive, pose, Controls::isRedAlliance);
  }

  public SetRobotPose(
      CommandSwerveDrivetrain swerveDrive, Pose2d pose, BooleanSupplier manualFlip) {
    m_swerveDrive = swerveDrive;
    m_pose = pose;
    m_booleanSupplier = manualFlip;

    addRequirements(m_swerveDrive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.seedFieldRelative(SimConstants.pathPlannerFlip(m_pose, m_booleanSupplier));
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
