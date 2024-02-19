package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ARM.AMP_STATE;
import frc.robot.constants.INTAKE.INTAKE_STATE;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoScore extends Command {
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;
  private final Intake m_intake;
  private final double m_percentOutput;
  private final double allowableError = 500; // in RPM

  public AutoScore(Shooter shooter, AmpShooter ampShooter, Intake intake, double percentOutput) {
    m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_intake = intake;
    m_percentOutput = percentOutput;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_shooter.setRPMOutput(RPM_SETPOINT.SPEAKER.get());

    if (m_shooter.getRpmMaster() >= RPM_SETPOINT.SPEAKER.get() - allowableError
        && m_shooter.getRpmFollower() >= RPM_SETPOINT.SPEAKER.get() - allowableError) {

      m_ampShooter.setPercentOutput(AMP_STATE.SCORE.get());
      m_intake.setSpeed(
          INTAKE_STATE.FRONT_ROLLER_INTAKING.get(), INTAKE_STATE.BACK_ROLLER_INTAKING.get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPercentOutput(0);
    m_ampShooter.setPercentOutput(0);
    m_intake.setSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
