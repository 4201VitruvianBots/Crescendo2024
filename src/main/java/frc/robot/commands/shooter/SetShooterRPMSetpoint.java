// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.subsystems.Shooter;

public class SetShooterRPMSetpoint extends Command {
  private final Shooter m_shooter;
  private final double m_RPMOutputBottom;
  private final double m_RPMOutputTop;
  private final double allowableError = RPM_SETPOINT.TOLERANCE.get(); // in RPM
  private final GenericHID m_hid;

  public SetShooterRPMSetpoint(
      Shooter shooter,
      CommandXboxController xboxController,
      double RPMOutputBottom,
      double RPMOutputTop) {
    m_shooter = shooter;
    m_hid = xboxController.getHID();
    m_RPMOutputBottom = RPMOutputBottom;
    m_RPMOutputTop = RPMOutputTop;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setNeutralMode(NeutralModeValue.Coast);
    m_shooter.setShooterState(true);
  }

  @Override
  public void execute() {
    // m_shooter.setRPMOutputFOC(m_RPMOutput);
    m_shooter.setRPMOutput(m_RPMOutputBottom, m_RPMOutputTop);
    if ((m_shooter.getRpmMaster() >= (m_RPMOutputBottom - allowableError))
        && (m_shooter.getRpmFollower() >= m_RPMOutputTop - allowableError)) {
      m_hid.setRumble(RumbleType.kBothRumble, 0.4);
    } else {
      m_hid.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPercentOutput(0);
    m_shooter.setShooterState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
