// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.UPTAKE.UPTAKE_STATE;
import frc.robot.subsystems.Uptake;

public class AutoRunUptake extends Command {
  Uptake m_uptake;
  UPTAKE_STATE m_state;

  public AutoRunUptake(Uptake uptake, UPTAKE_STATE state) {
    m_uptake = uptake;
    m_state = state;

    addRequirements(m_uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_uptake.setPercentOutput(m_state.get());
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
