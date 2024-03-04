// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.ROBOT;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmControlMode extends InstantCommand {
  private Arm m_arm;
  private ROBOT.CONTROL_MODE m_controlMode;

  public SetArmControlMode(Arm arm, ROBOT.CONTROL_MODE controlMode) {
    m_arm = arm;
    m_controlMode = controlMode;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setControlMode(m_controlMode);
  }
}
