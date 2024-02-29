// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

public class ToggleClimbMode extends InstantCommand {
  private final Climber m_climber;
  private final Arm m_arm;

  /** Creates a new ToggleClimberControlMode. */
  public ToggleClimbMode(Climber climber, Arm arm) {
    m_climber = climber;
    m_arm = arm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber, m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setClimbState(!m_climber.getClimbState());
    m_arm.setControlMode(CONTROL_MODE.OPEN_LOOP);
    //    ROBOT.CONTROL_MODE climberControlMode = m_climber.getClosedLoopControlMode();

    //    switch (climberControlMode) {
    //      case OPEN_LOOP:
    //        m_climber.setClosedLoopControlMode(CONTROL_MODE.CLOSED_LOOP);
    //        m_climber.resetTrapezoidState();
    //        m_climber.holdClimber();
    //        break;
    //      default:
    //      case CLOSED_LOOP:
    //        m_climber.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
    //        break;
    //    }
  }

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
