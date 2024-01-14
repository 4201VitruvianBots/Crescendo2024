/* Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project. */

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpFlipper;
import java.util.function.DoubleSupplier;

public class AmpFlipperJoystick extends Command {
  /** Creates a new AmpFlipperForward. */
  AmpFlipper m_flipper;

  private DoubleSupplier m_output;

  public AmpFlipperJoystick(AmpFlipper flipper, DoubleSupplier output) {
    m_flipper = flipper;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_flipper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flipper.setPercentOutput(m_output.getAsDouble());
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
