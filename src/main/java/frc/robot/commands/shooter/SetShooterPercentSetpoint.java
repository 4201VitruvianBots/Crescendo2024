package frc.robot.commands.shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class SetShooterPercentSetpoint extends Command {
  private final Shooter m_shooter;
  private final double m_percentOutput;
    private final GenericHID m_hid;

  public SetShooterPercentSetpoint(Shooter shooter, double percentOutput, CommandXboxController xboxController) {
    m_shooter = shooter;
    m_percentOutput = percentOutput;
    m_hid = xboxController.getHID();
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setNeutralMode(NeutralModeValue.Coast);
    m_shooter.setShooterState(true);

  }

  @Override
  public void execute() {
    m_shooter.setPercentOutput(m_percentOutput);
     if ((m_shooter.getRpmMaster() >= 7000) && (m_shooter.getRpmFollower() >= 7000)) {
      m_hid.setRumble(RumbleType.kBothRumble, 1);
    } else {
      m_hid.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setPercentOutput(0);
    m_shooter.setShooterState(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
