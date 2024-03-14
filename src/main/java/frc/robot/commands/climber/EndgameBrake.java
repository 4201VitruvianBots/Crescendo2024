// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EndgameBrake extends InstantCommand {
  private final Climber m_climber;
  private final Arm m_arm;
  private final AmpShooter m_ampShooter;
  
  public EndgameBrake(Climber climber, Arm arm, AmpShooter ampShooter) {
    m_climber = climber;
    m_arm = arm;
    m_ampShooter = ampShooter;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.holdClimber();
    m_climber.setClimberNeutralMode(NeutralModeValue.Brake);
    m_arm.resetMotionMagicState();
    m_arm.setNeutralMode(NeutralModeValue.Brake);
    m_ampShooter.setAutoPercentOutput(0);
    m_ampShooter.setPercentOutput(0);
    m_ampShooter.setNeutralMode(NeutralModeValue.Brake);
  }
}
