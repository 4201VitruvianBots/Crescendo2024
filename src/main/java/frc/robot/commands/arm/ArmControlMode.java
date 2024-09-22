// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT.CONTROL_MODE;
import frc.robot.subsystems.Arm;

public class ArmControlMode extends InstantCommand {
  private final Arm m_arm;
  private CONTROL_MODE m_controlMode;

  /** Set control mode directly */
  public ArmControlMode(Arm arm, CONTROL_MODE controlMode) {
    m_arm = arm;
    m_controlMode = controlMode;
  }
  
  /** Toggle control mode */
  public ArmControlMode(Arm arm) {
    this(arm, arm.getControlMode() == CONTROL_MODE.CLOSED_LOOP ? CONTROL_MODE.OPEN_LOOP : CONTROL_MODE.CLOSED_LOOP);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    m_arm.setControlMode(m_controlMode);
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
