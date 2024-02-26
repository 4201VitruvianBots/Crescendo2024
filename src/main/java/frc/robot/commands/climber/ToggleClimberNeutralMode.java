// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ToggleClimberNeutralMode extends Command {
  /** Creates a new ToggleElevatorCoastMode. */
  private final Climber m_climber;

  public ToggleClimberNeutralMode(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NeutralModeValue neutralMode = m_climber.getNeutralMode();
    if (neutralMode == NeutralModeValue.Coast) {
      m_climber.setClimberNeutralMode(NeutralModeValue.Brake);
    } else if (neutralMode == NeutralModeValue.Brake) {
      m_climber.setClimberNeutralMode(NeutralModeValue.Coast);
    }
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
    return true;
  }
}
