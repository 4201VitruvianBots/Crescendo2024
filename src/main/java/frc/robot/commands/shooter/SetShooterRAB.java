// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;

public class SetShooterRAB extends Command {
  private final Shooter m_shooter;
  private final DoubleSupplier m_triggerSpeed;
  private final DoubleSupplier m_RPMOutputTop;
  private final double m_minimumRPM;
  private final double m_maximumRPM;

  public SetShooterRAB(
      Shooter shooter,
      DoubleSupplier triggerSpeed,
      double minimumRPM,
      double maximumRPM,
      DoubleSupplier RPMOutputTop) {
    m_shooter = shooter;
    m_triggerSpeed = triggerSpeed;
    m_RPMOutputTop = RPMOutputTop;
    m_minimumRPM = minimumRPM;
    m_maximumRPM = maximumRPM;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void execute() {
    double RPMOutputBottom =
        (MathUtil.applyDeadband(m_triggerSpeed.getAsDouble(), 0.05) * (m_maximumRPM - m_minimumRPM)) + m_minimumRPM;

    m_shooter.setShootingState(RPMOutputBottom > 0);
    m_shooter.setRPMOutputFOC(RPMOutputBottom);
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPercentOutput(0);
    m_shooter.setShootingState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
