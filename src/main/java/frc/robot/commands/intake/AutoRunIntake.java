// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoRunIntake extends Command {
  /** Creates a new AutoRunIntake. */
  Intake m_intake;

  double m_speed;
  double m_speed2;

  public AutoRunIntake(Intake intake, double speed, double speed2) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_speed = speed;
    m_speed2 = speed2;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakeState(true);
    m_intake.setSpeed(m_speed, m_speed2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_intake.checkEitherIntakeSensor();
    return true;
  }
}
