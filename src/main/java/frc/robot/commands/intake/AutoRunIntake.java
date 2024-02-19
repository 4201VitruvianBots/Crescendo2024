// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.INTAKE.INTAKE_STATE;
import frc.robot.subsystems.Intake;

public class AutoRunIntake extends Command {
  /** Creates a new AutoRunIntake. */
  private final Intake m_intake;

  private final INTAKE_STATE m_frontState;
  private final INTAKE_STATE m_backState;

  public AutoRunIntake(Intake intake, INTAKE_STATE frontState, INTAKE_STATE backState) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_frontState = frontState;
    m_backState = backState;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setSpeed(m_frontState.get(), m_backState.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_intake.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.getSensorInput1() || m_intake.getSensorInput1();
  }
}
