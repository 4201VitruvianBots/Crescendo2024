// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterRPMSetpoint extends Command {
  private final Shooter m_shooter;
  private final double m_RPMOutputBottom;
  private final double m_RPMOutputTop;

  public SetShooterRPMSetpoint(Shooter shooter, double RPMOutputBottom, double RPMOutputTop) {
    m_shooter = shooter;
    m_RPMOutputBottom = RPMOutputBottom;
    m_RPMOutputTop = RPMOutputTop;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_shooter.setNeutralMode(NeutralModeValue.Coast);
    }

  @Override
  public void execute() {
    // m_shooter.setRPMOutputFOC(m_RPMOutput);
    m_shooter.setRPMOutput(m_RPMOutputBottom, m_RPMOutputTop);
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
