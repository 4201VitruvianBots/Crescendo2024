// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ToggleShooterTestMode extends CommandBase {
  /** Creates a new ToggleElevatorTestMode. */
  private final Shooter m_shooter;

  private final Command m_defaultCommand;

  private boolean m_testMode = false;

  public ToggleShooterTestMode(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;

    m_defaultCommand = m_shooter.getDefaultCommand();

    addRequirements(m_shooter);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_testMode = !m_testMode;

    m_shooter.setTestMode(m_testMode);

    if (m_testMode) {
      m_shooter.setDefaultCommand(new RunShooterTestMode(m_shooter));
    } else {
      m_shooter.setDefaultCommand(m_defaultCommand);
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
