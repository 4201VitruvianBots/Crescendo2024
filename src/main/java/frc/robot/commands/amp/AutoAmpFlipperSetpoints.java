// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AMP.FLIPPER_SETPOINT;
import frc.robot.subsystems.AmpFlipper;

public class AutoAmpFlipperSetpoints extends Command {
  AmpFlipper m_flipper;
  FLIPPER_SETPOINT m_state;

  /** Creates a new AmpFlipperForward. */
  public AutoAmpFlipperSetpoints(AmpFlipper flipper, FLIPPER_SETPOINT state) {
    m_flipper = flipper;
    m_state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flipper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_flipper.setDesiredSetpointRadians(m_state.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
