package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoSetPercentOutput extends Command {
  Shooter m_shooter;
  double m_percentOutput;

  public AutoSetPercentOutput(Shooter shooter, double percentOutput) {
    m_shooter = shooter;
    m_percentOutput = percentOutput;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setPercentOutput(m_percentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
