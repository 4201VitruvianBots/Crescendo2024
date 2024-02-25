package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Shooter;

public class DriverScore extends Command {
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;
  private double m_AmpPercentOutput;
  private double m_RPMOutput;
  private final double allowableError = 350; // in RPM

  public DriverScore(
      Shooter shooter, AmpShooter ampShooter, double AmpPercentOutput, double RPMOutput) {
    m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_AmpPercentOutput = AmpPercentOutput;
    m_RPMOutput = RPMOutput;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if ((m_shooter.getRpmMaster() >= (m_RPMOutput - allowableError))
        && (m_shooter.getRpmFollower() >= (m_RPMOutput - allowableError))) {

      m_ampShooter.setPercentOutput(m_AmpPercentOutput);
    }
  }

  // attempt shot even though RPM is low

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ampShooter.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
