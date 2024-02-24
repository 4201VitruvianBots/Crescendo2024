/* Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project. */

package frc.robot.commands.amp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ARM;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class ArmJoystickSetpoint extends Command {
  /** Creates a new ArmForward. */
  private final Arm m_arm;

  private final DoubleSupplier m_output;

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
    double m_setpointDeadband = MathUtil.applyDeadband(m_output.getAsDouble(), 0.05);

    if (m_setpointDeadband != 0.0) {
      var rotationSetpoint =
          MathUtil.clamp(
              m_setpointDeadband * 0.5 + m_arm.getCurrentRotation(),
              Units.degreesToRotations(ARM.minAngleDegrees),
              Units.degreesToRotations(ARM.maxAngleDegrees));
      m_arm.setDesiredSetpointRotations(rotationSetpoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setDesiredSetpointRotations(Units.degreesToRotations(m_arm.getCurrentRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
