// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CLIMBER.CLIMBSTATE;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

public class SetClimberOutput extends Command {
  private final Climber m_climber;
  private DoubleSupplier m_input;

  private double input = 0;
  private boolean latched;
  private CLIMBSTATE state = CLIMBSTATE.STILL;

  /** Creates a new SetClimberOutput. */
  public SetClimberOutput(Climber climber, DoubleSupplier input) {
    m_climber = climber;
    m_input = input;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climber.getClimbState()) {
      input = Math.abs(m_input.getAsDouble()) > 0.2 ? -m_input.getAsDouble() : 0;

      state = ((input == 0) ? CLIMBSTATE.STILL : CLIMBSTATE.MOVING);
      switch (state) {
        case MOVING:
          m_climber.setPercentOutput(input, false);
          latched = false;
          break;
        case STILL:
        default:
          if (!latched) {
            m_climber.holdClimber();
            latched = true;
          }
          m_climber.holdClimber();
          break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
