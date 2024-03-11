package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Shooter;

public class DriverScore extends Command {
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;
  private final double m_ampPercentOutput;
  private final double m_rpmOutput;
  private final double allowableError = RPM_SETPOINT.TOLERANCE.get(); // in RPM

  public DriverScore(
      Shooter shooter, AmpShooter ampShooter, double rpmOutput, double ampPercentOutput) {
    m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_rpmOutput = rpmOutput;
    m_ampPercentOutput = ampPercentOutput;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if ((m_shooter.getRpmMaster() >= (m_rpmOutput - allowableError))
        && (m_shooter.getRpmFollower() >= (m_rpmOutput - allowableError))) {

      m_ampShooter.setPercentOutput(m_ampPercentOutput);
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
