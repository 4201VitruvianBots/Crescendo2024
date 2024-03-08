package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VISION.TARGET_STATE;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTargetState extends Command {
  private final CommandSwerveDrivetrain m_swerveDriveTrain;
  private final TARGET_STATE m_state;

  public SetTargetState(CommandSwerveDrivetrain swerveDriveTrain, TARGET_STATE state) {
    m_swerveDriveTrain = swerveDriveTrain;
    m_state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDriveTrain.setTargetState(m_state);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDriveTrain.setTargetState(TARGET_STATE.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
