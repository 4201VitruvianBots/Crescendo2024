// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

public class RunClimberJoystick extends Command {
  /** Creates a new RunElevatorJoystick. This is our default command */
  private final Climber m_climber;

  private final DoubleSupplier m_joystickY;

  public RunClimberJoystick(Climber climber, DoubleSupplier joystickY) {
    m_climber = climber;
    m_joystickY = joystickY;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Adds a Deadband so joystick Ys below 0.05 won't be registered
    double joystickYDeadbandOutput = MathUtil.applyDeadband(m_joystickY.getAsDouble(), 0.1);

    if (joystickYDeadbandOutput != 0.0) {
      m_climber.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
      m_climber.setJoystickY(-joystickYDeadbandOutput);
      m_climber.setClimbState(true);
    }
    if (joystickYDeadbandOutput == 0
        && m_climber.getClosedLoopControlMode() == CONTROL_MODE.OPEN_LOOP) {
      m_climber.setDesiredPositionMeters(m_climber.getHeightMeters());
      m_climber.resetTrapezoidState();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimbState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}