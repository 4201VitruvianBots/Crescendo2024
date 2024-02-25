package frc.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class DefaultFlywheel extends Command {
  private final Shooter m_shooter;

  public DefaultFlywheel(Shooter shooter) {
    m_shooter = shooter;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // If no other command is running, slow down the flywheel to decelerate it faster
    // This function is causing CommandScheduler loop overruns (~0.09 seconds per loop, ~0.07 seconds).
    // The overrun then gets transferred to ParallelRaceGroup.initialize() once this is commented out. TODO: fix
    if (m_shooter.getRpmMaster() > 500) {
      m_shooter.setPercentOutput(-0.01);
      m_shooter.setNeutralMode(NeutralModeValue.Brake);
    } else {
      m_shooter.setPercentOutput(0);
      m_shooter.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
