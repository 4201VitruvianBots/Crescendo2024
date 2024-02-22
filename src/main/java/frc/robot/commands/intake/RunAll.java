// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RunAll extends Command {
  Intake m_intake;
  AmpShooter m_ampShooter;
  double m_speed;
  Shooter m_shooter;
  double m_speed2;
  double m_ampSpeed;
  double m_rpm;

  /** Creates a new RunIntake. */
  public RunAll(
      Intake intake,
      Shooter shooter,
      AmpShooter ampShooter,
      double speed,
      double speed2,
      double ampSpeed,
      double RPM) {
    m_intake = intake;
    m_ampShooter = ampShooter;
    m_shooter = shooter;
    m_speed = speed;
    m_speed2 = speed2;
    m_ampSpeed = ampSpeed;
    m_rpm = RPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_ampShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setSpeed(m_speed, m_speed2);
    m_ampShooter.setPercentOutput(m_ampSpeed);
    m_shooter.setRPMOutput(m_rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setSpeed(0,0);
    m_ampShooter.setPercentOutput(0);
    m_shooter.setRPMOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
