// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetPercentOutput extends Command {
  private final Intake m_intake;
  private final double m_percentOutput1;
  private final double m_percentOutput2;

  /** Creates a new idk. */
  public SetPercentOutput(Intake intake, double percentOutput1, double percentOutput2) {
    m_intake = intake;
    m_percentOutput1 = percentOutput1;
    m_percentOutput2 = percentOutput2;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setMotorOutput1(m_percentOutput1);
    m_intake.setMotorOutput2(m_percentOutput2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotorOutput1(0);
    m_intake.setMotorOutput2(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
