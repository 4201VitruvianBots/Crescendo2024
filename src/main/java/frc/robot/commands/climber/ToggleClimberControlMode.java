// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Climber;

public class ToggleClimberControlMode extends InstantCommand {
  private final Climber m_climber;

  /** Creates a new ToggleClimberControlMode. */
  public ToggleClimberControlMode(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_climber.getClosedLoopControlMode() == CONTROL_MODE.OPEN_LOOP)
      m_climber.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
    else if (m_climber.getClosedLoopControlMode() == CONTROL_MODE.CLOSED_LOOP)
      m_climber.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
  }
}
