// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ARM;
import frc.robot.constants.ARM.ARM_SETPOINT;
import frc.robot.subsystems.Arm;

public class ArmSetpoint extends Command {
  private final Arm m_arm;
  private ARM.ARM_SETPOINT m_setpoint;

  /** Creates a new ArmForward. */
  public ArmSetpoint(Arm arm, ARM_SETPOINT setpoint) {
    m_arm = arm;
    m_setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setDesiredSetpointRotations(m_setpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setDesiredSetpointRotations(ARM.ARM_SETPOINT.STOWED.get());
    m_arm.setArmState(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
