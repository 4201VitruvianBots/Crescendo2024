// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SHOOTER;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoRunAmpTakeTwo extends Command {
  /** Creates a new AutoRunIntake. */
  Intake m_intake;

  AmpShooter m_ampShooter;
  Shooter m_shooter;

  double m_speed;
  double m_speed2;
  double m_ampSpeed;

  double startTime;
  boolean readyToFire;
  boolean sensorClear;

  public AutoRunAmpTakeTwo(
      Intake intake,
      AmpShooter ampShooter,
      double speed,
      double speed2,
      double ampSpeed,
      Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_ampShooter = ampShooter;
    m_speed = speed;
    m_speed2 = speed2;
    m_ampSpeed = ampSpeed;
    m_shooter = shooter;
    addRequirements(m_intake, m_ampShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setSpeed(m_speed, m_speed2);

    m_ampShooter.setPercentOutput(0);

    readyToFire = false;
    sensorClear = false;
    startTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!readyToFire) {
      if (m_shooter.getRpmMaster()
              >= (m_shooter.getBottomRPMsetpoint() - SHOOTER.RPM_SETPOINT.TOLERANCE.get())
          && m_shooter.getRpmFollower()
              >= (m_shooter.getTopRPMsetpoint() - SHOOTER.RPM_SETPOINT.TOLERANCE.get())) {
        readyToFire = true;
      }
    }

    if (readyToFire) {
      m_ampShooter.setPercentOutput(m_ampSpeed);

      if (!m_intake.getSensorInput1() && !m_intake.getSensorInput2() && !sensorClear) {

        startTime = Timer.getFPGATimestamp();
        sensorClear = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ampShooter.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sensorClear && (Timer.getFPGATimestamp() - startTime) > 0.5;
  }
}
