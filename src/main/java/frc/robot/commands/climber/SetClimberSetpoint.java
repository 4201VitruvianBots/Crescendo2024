// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CLIMBER;
import frc.robot.constants.CLIMBER.CLIMBER_SETPOINT;
import frc.robot.subsystems.Climber;

public class SetClimberSetpoint extends Command {
  private final Climber m_climber;
  private final CLIMBER_SETPOINT m_setpoint;

  /** Creates a new SetClimberSetpoint. */
  public SetClimberSetpoint(Climber climber, CLIMBER_SETPOINT setpoint) {
    m_climber = climber;
    m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setDesiredPositionMeters(m_setpoint.getSetpointMeters());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setDesiredSetpoint(CLIMBER.CLIMBER_SETPOINT.FULL_RETRACT.getSetpointMeters());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
