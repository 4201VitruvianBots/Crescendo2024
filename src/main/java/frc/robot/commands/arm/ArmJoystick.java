/* Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project. */

package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ARM;
import frc.robot.constants.ROBOT;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class ArmJoystick extends Command {
  /** Creates a new ArmForward. */
  private final Arm m_arm;

  private final DoubleSupplier m_output;

  public ArmJoystick(Arm arm, DoubleSupplier output) {
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
    double m_joystickDeadband = MathUtil.applyDeadband(Math.pow(m_output.getAsDouble(), 3), 0.05);

    if (m_joystickDeadband != 0.0) {
      if (m_arm.getControlMode() == ROBOT.CONTROL_MODE.CLOSED_LOOP) {
        var rotationSetpoint =
            MathUtil.clamp(
                m_joystickDeadband * 0.5 + m_arm.getCurrentRotation(),
                Units.degreesToRotations(ARM.minAngleDegrees),
                Units.degreesToRotations(ARM.maxAngleDegrees));
        m_arm.setDesiredSetpointRotations(rotationSetpoint);
      } else if (m_arm.getControlMode() == ROBOT.CONTROL_MODE.OPEN_LOOP) {
        if (ARM.limitOpenLoop) {
          // Upper limit
          if (m_arm.getCurrentAngle() >= ARM.maxAngleDegrees - 1)
            m_joystickDeadband = Math.min(m_joystickDeadband, 0);

          // Lower limit
          if (m_arm.getCurrentAngle() <= ARM.minAngleDegrees + 1)
            m_joystickDeadband = Math.max(m_joystickDeadband, 0);
        }
        m_arm.setPercentOutput(m_joystickDeadband * ARM.joystickMultiplier);
      }
    } else {
      if (m_arm.getControlMode() == ROBOT.CONTROL_MODE.OPEN_LOOP)
        m_arm.setPercentOutput(m_joystickDeadband * ARM.joystickMultiplier);
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
