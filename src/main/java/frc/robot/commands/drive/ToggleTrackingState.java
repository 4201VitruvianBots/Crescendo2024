// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.VISION;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ToggleTrackingState extends InstantCommand {
  private final CommandSwerveDrivetrain m_swerveDrive;

  private final VISION.TRACKING_STATE m_state;

  public ToggleTrackingState(CommandSwerveDrivetrain swerveDrive, VISION.TRACKING_STATE state) {
    m_swerveDrive = swerveDrive;
    m_state = state;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_swerveDrive.getTrackingState() == TRACKING_STATE.NONE) { 
        m_swerveDrive.setTrackingState(m_state);
    } else if (m_swerveDrive.getTrackingState() != m_state) {
        m_swerveDrive.setTrackingState(m_state);
    } else {
        m_swerveDrive.setTrackingState(TRACKING_STATE.NONE);
    }
  }
  
}
