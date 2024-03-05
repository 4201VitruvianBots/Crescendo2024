package frc.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoScore extends Command {
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;
  private final Intake m_intake;
  private double m_AmpPercentOutput;
  private double m_RPMOutput;
  private final double m_FrontIntakePercentOutput;
  private final double m_BackIntakeAmpPercentOutput;
  private final double allowableError = 350; // in RPM
  private final double m_timeToShoot;
  private final double m_withTimeout;
  private final double reverseTimerThreshold = 0.25;

  private final double minRPMThreshold = 300;

  private final boolean funny = true;

  private final Timer m_timer = new Timer();
  private final Timer m_reversetimer = new Timer();
  private final Timer m_shoottimer = new Timer();

  public AutoScore(
      Shooter shooter,
      AmpShooter ampShooter,
      Intake intake,
      double AmpPercentOutput,
      double RPMOutput,
      double FrontIntakeAmpPercentOutput,
      Double BackIntakeAmpPercentOutput,
      double TimeToShoot,
      double withTimeout) {
    m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_intake = intake;
    m_AmpPercentOutput = AmpPercentOutput;
    m_FrontIntakePercentOutput = FrontIntakeAmpPercentOutput;
    m_BackIntakeAmpPercentOutput = BackIntakeAmpPercentOutput;
    m_RPMOutput = RPMOutput;
    m_timeToShoot = TimeToShoot;
    m_withTimeout = withTimeout;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setNeutralMode(NeutralModeValue.Coast);
    m_timer.stop();
    m_timer.reset();
    m_reversetimer.stop();
    m_reversetimer.reset();
    m_shoottimer.stop();
    m_shoottimer.reset();
  }

  @Override
  public void execute() {
    m_shooter.setRPMOutput(m_RPMOutput);
    m_timer.start();

    if ((m_shooter.getRpmMaster() >= (m_RPMOutput - allowableError))
        && (m_shooter.getRpmFollower() >= (m_RPMOutput - allowableError))) {

      m_ampShooter.setPercentOutput(-m_AmpPercentOutput);

      m_reversetimer.start();

      if (m_reversetimer.hasElapsed(reverseTimerThreshold)) {
        m_ampShooter.setPercentOutput(m_AmpPercentOutput);
        m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);
        m_shoottimer.start();
      }
    } else if (m_timer.hasElapsed(m_withTimeout)) {

      if ((m_shooter.getRpmMaster() >= minRPMThreshold)
          || (m_shooter.getRpmFollower() >= minRPMThreshold)) {
        System.err.println("Flywheel Did not Reach Setpoint RPM");
        SmartDashboard.putString("isFlyWheelMoving?", "Flywheel Did not Reach Setpoint RPM");
        m_ampShooter.setPercentOutput(-m_AmpPercentOutput);

        m_reversetimer.start();

        if (m_reversetimer.hasElapsed(reverseTimerThreshold)) {
          m_ampShooter.setPercentOutput(m_AmpPercentOutput);
          m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);
          m_shoottimer.start();
        }
      }

      if ((m_shooter.getRpmMaster() < minRPMThreshold)
          || (m_shooter.getRpmFollower() < minRPMThreshold)) {

        m_ampShooter.setPercentOutput(-m_AmpPercentOutput);
        m_intake.setSpeed(-m_FrontIntakePercentOutput, -m_BackIntakeAmpPercentOutput);
        m_shoottimer.start();

        System.err.println("Flywheel Did not Move");
        SmartDashboard.putString("isFlyWheelMoving?", "Flywheel Did not Move");

        // Abort Shot bc flywheel is broken

      }

      // attempt shot even though RPM is low

    }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ampShooter.setPercentOutput(0);
    m_intake.setSpeed(0, 0);
    m_timer.stop();
    m_timer.reset();
    m_reversetimer.stop();
    m_reversetimer.reset();
    m_shoottimer.stop();
    m_shoottimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shoottimer.hasElapsed(m_timeToShoot);
  }
}
