// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.CLIMBER;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

public class RunClimberJoystick extends Command {
  /** Creates a new RunElevatorJoystick. This is our default command */
  private final Climber m_climber;

  private final DoubleSupplier m_joystickY;

  private final CommandXboxController m_xboxController;

  public RunClimberJoystick(
      Climber climber, DoubleSupplier joystickY, CommandXboxController xboxController) {
    m_climber = climber;
    m_joystickY = joystickY;
    m_xboxController = xboxController;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Adds a Deadband so joystick Ys below 0.05 won't be registered
    if (m_climber.getClimbState()) {
      double joystickYDeadbandOutput =
          MathUtil.applyDeadband(Math.pow(m_joystickY.getAsDouble(), 3), 0.1);

      if (joystickYDeadbandOutput != 0.0) {
        if (joystickYDeadbandOutput < 0)
          joystickYDeadbandOutput *= CLIMBER.kLimitedPercentOutputMultiplier;
        m_climber.setJoystickY(-joystickYDeadbandOutput);
        m_climber.setClimbState(true);
      }
      if (joystickYDeadbandOutput == 0) {
        m_climber.holdClimber();
        m_climber.setPercentOutput(0);
      }
    } else {
      m_climber.holdClimber();
      m_climber.setClimberNeutralMode(NeutralModeValue.Brake);
    }

    if (m_climber.getAvgCurrentDraw() >= 30) {
      m_xboxController.getHID().setRumble(RumbleType.kBothRumble, 0.2);
    } else {
      m_xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.holdClimber();
    m_climber.setClimberNeutralMode(NeutralModeValue.Brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
