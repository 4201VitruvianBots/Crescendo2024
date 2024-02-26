// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ResetClimberHeight extends Command {
  private final Climber m_climber;
  private final double m_meters;

  /** Creates a new ResetClimberHeight. */
  public ResetClimberHeight(Climber climber, double meters) {
    m_climber = climber;
    m_meters = meters;
    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(m_climber);
  }

  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setSensorPosition(m_meters);
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
    return false;
  }
}
