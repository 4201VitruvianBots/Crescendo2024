// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;

public class AutoAmpIntake extends Command {
  private final Intake m_intake;
  private final AmpShooter m_ampShooter;

  private final double m_speed;
  private final double m_speed2;
  private final double m_ampSpeed;

  /** Creates a new RunIntake. */
  public AutoAmpIntake(
      Intake intake, double speed, double speed2, AmpShooter ampShooter, double ampSpeed) {
    m_intake = intake;
    m_ampShooter = ampShooter;
    m_speed = speed;
    m_speed2 = speed2;
    m_ampSpeed = ampSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_ampShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if ((m_intake.checkEitherIntakeSensorActive())) {

      m_intake.setSpeed(m_speed, m_speed2);
      m_ampShooter.setAutoPercentOutput(m_ampSpeed);
    } else m_intake.setSpeed(m_speed, m_speed2);
    m_ampShooter.setAutoPercentOutput(0);
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
