package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoSetRPMSetpoint extends Command {
  private final Shooter m_shooter;
  private final double m_RPM;

  public AutoSetRPMSetpoint(Shooter shooter, double RPM) {
    m_shooter = shooter;
    m_RPM = RPM;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setRPMOutput(m_RPM);
  }

  @Override
  public void execute() {}

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
