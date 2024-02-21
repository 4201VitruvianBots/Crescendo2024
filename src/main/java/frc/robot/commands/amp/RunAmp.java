// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;

public class RunAmp extends Command {
  private final AmpShooter m_ampShooter;
  private final Intake m_intake;
  private final double m_percentOutput;

  public RunAmp(AmpShooter ampShooter, Intake intake, double percentOutput) {
    m_ampShooter = ampShooter;
    m_intake = intake;
    m_percentOutput = percentOutput;

    addRequirements(m_ampShooter, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_ampShooter.setPercentOutput(m_percentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ampShooter.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
