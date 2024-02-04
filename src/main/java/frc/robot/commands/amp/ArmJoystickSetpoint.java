/* Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project. */

package frc.robot.commands.amp;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class ArmJoystickSetpoint extends Command {
  /** Creates a new ArmForward. */
  Arm m_arm;

  private DoubleSupplier m_output;

  public ArmJoystickSetpoint(Arm arm, DoubleSupplier output) {
    m_arm = arm;
    m_output = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setDesiredSetpointRotations(
        m_output.getAsDouble() * 0.1 + Units.degreesToRotations(m_arm.getAngleDegrees()));
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
