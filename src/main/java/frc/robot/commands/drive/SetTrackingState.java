// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VISION;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class SetTrackingState extends Command {
  /** Creates a new RestGyro. */
  private final CommandSwerveDrivetrain m_swerveDrive;

  private final Vision m_vision;
  private final VISION.TRACKING_STATE m_state;

  public SetTrackingState(
      CommandSwerveDrivetrain swerveDrive, Vision vision, VISION.TRACKING_STATE state) {
    m_swerveDrive = swerveDrive;
    m_vision = vision;
    m_state = state;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_state == VISION.TRACKING_STATE.NOTE) {
      if (m_vision.hasGamePieceTarget()) {
        m_swerveDrive.setTrackingState(m_state);
      } else {
        m_swerveDrive.setTrackingState(VISION.TRACKING_STATE.NONE);
      }
    } else m_swerveDrive.setTrackingState(m_state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.setTrackingState(VISION.TRACKING_STATE.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
