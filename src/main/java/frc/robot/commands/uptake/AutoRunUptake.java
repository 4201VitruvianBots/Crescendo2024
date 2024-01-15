package frc.robot.commands.uptake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Uptake;
import frc.robot.constants.UPTAKE.UPTAKE_STATE;

public class AutoRunUptake extends Command {
  Uptake m_uptake;
  UPTAKE_STATE m_state;

  /** Creates a new RunIntake. 
 * @return */
  public void RunUptake(Uptake uptake, UPTAKE_STATE state) {
    m_uptake = uptake;
    m_state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_uptake.setSpeed(m_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_uptake.setSpeed(0.3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
