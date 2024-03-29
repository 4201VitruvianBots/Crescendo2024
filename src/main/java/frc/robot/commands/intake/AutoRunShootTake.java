// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;

public class AutoRunShootTake extends Command {
  /** Creates a new AutoRunIntake. */
  Intake m_intake;

  AmpShooter m_ampShooter;

  double m_speed;
  double m_speed2;
  double m_ampSpeed;
  double m_timerThreshold;

  private final Timer m_timer = new Timer();

  public AutoRunShootTake(
      Intake intake,
      AmpShooter ampShooter,
      double speed,
      double speed2,
      double ampSpeed,
      double TimerThreshold) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_speed = speed;
    m_speed2 = speed2;
    m_ampSpeed = ampSpeed;
    m_ampShooter = ampShooter;
    m_timerThreshold = TimerThreshold;
    addRequirements(m_intake, m_ampShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_timer.hasElapsed(m_timerThreshold)) {
      m_ampShooter.setPercentOutput(m_ampSpeed);

      m_intake.setSpeed(m_speed, m_speed2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_intake.getSensorInput1() || m_intake.getSensorInput2();

    return false;
  }
}
