package frc.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SHOOTER;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Exhibition extends Command {
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;
  private final Intake m_intake;
  private final double m_AmpPercentOutput;
  private final double m_RPMOutput;
  private final double m_FrontIntakePercentOutput;
  private final double m_BackIntakeAmpPercentOutput;


  public Exhibition(
      Shooter shooter,
      AmpShooter ampShooter,
      Intake intake,
      double AmpPercentOutput,
      double RPMOutput,
      double FrontIntakeAmpPercentOutput,
      Double BackIntakeAmpPercentOutput) {
    m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_intake = intake;
    m_AmpPercentOutput = AmpPercentOutput;
    m_FrontIntakePercentOutput = FrontIntakeAmpPercentOutput;
    m_BackIntakeAmpPercentOutput = BackIntakeAmpPercentOutput;
    m_RPMOutput = RPMOutput;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_shooter.setRPMOutput(m_RPMOutput);
      m_ampShooter.setPercentOutput(m_AmpPercentOutput);
        m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);
    
  



      // attempt shot even though RPM is low

    
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ampShooter.setPercentOutput(0);
    m_intake.setSpeed(0, 0);
    m_shooter.setRPMOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
