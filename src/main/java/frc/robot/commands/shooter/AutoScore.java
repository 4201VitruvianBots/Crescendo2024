package frc.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoScore extends Command {
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;
  private final Intake m_intake;
  private final double allowableError = 350; // in RPM
  private final double m_AmpPercentOutput;
  private final double m_FrontIntakePercentOutput;
  private final double m_BackIntakeAmpPercentOutput;
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
    m_timeToShoot = TimeToShoot;
    m_withTimeout = withTimeout;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }

  @Override
  public void execute() {
      m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);

    

    if ((m_shooter.getRpmMaster() >= (RPM_SETPOINT.MAX.get()-allowableError))
        && (m_shooter.getRpmFollower() >= ( RPM_SETPOINT.MAX.get() -allowableError))) {

     
        m_ampShooter.setPercentOutput(m_AmpPercentOutput);
      
    

      // attempt shot even though RPM is low

    }
    else         m_ampShooter.setPercentOutput(m_AmpPercentOutput);

  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ampShooter.setPercentOutput(0);
    m_intake.setSpeed(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
