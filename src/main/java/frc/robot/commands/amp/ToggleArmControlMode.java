// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleArmControlMode extends InstantCommand {
  private final Arm m_arm;

  public ToggleArmControlMode(Arm arm) {
    m_arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ROBOT.CONTROL_MODE armControlMode = m_arm.getControlMode();

    if (armControlMode == ROBOT.CONTROL_MODE.CLOSED_LOOP) {
      m_arm.setControlMode(ROBOT.CONTROL_MODE.OPEN_LOOP);
    } else if (armControlMode == ROBOT.CONTROL_MODE.OPEN_LOOP) {
      m_arm.setControlMode(ROBOT.CONTROL_MODE.CLOSED_LOOP);
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
