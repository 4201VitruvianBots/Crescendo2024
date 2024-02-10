// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterRPMSetpoint extends Command {
  Shooter m_shooter;
  double m_RPMOutput;

  public SetShooterRPMSetpoint(Shooter shooter, double RPMOutput) {
    m_shooter = shooter;
    m_RPMOutput = RPMOutput;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setRpmOutput(m_RPMOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setRpmOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}