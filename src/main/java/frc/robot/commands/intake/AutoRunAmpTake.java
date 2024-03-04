// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;

public class AutoRunAmpTake extends Command {
  /** Creates a new AutoRunIntake. */
  Intake m_intake;

  double m_speed;
  double m_speed2;
  AmpShooter m_ampShooter;
  double m_ampSpeed;

  public AutoRunAmpTake(
      Intake intake, AmpShooter ampShooter, double speed, double speed2, double ampSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_speed = speed;
    m_speed2 = speed2;
    m_ampSpeed = ampSpeed;
    m_ampShooter = ampShooter;
    addRequirements(m_intake, m_ampShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setSpeed(m_speed, m_speed2);

    m_ampShooter.setPercentOutput(m_ampSpeed);
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
    // return m_intake.getSensorInput1() || m_intake.getSensorInput2();
    return true;
  }
}
