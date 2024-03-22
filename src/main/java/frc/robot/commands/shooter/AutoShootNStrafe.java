// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AMPSHOOTER.STATE;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class AutoShootNStrafe extends Command {

  private final CommandSwerveDrivetrain m_swerveDrive;
  private final AmpShooter m_ampShooter;
  private final Intake m_intake;

  private final Timer m_readyTimer = new Timer();
  private final Timer m_shotTimer = new Timer();

  private final double m_timeToShoot = 0.75;

  boolean hasNote;

  public AutoShootNStrafe(
      CommandSwerveDrivetrain swerveDrive, AmpShooter ampShooter, Intake intake) {

    m_swerveDrive = swerveDrive;
    m_ampShooter = ampShooter;
    m_intake = intake;

    addRequirements(m_swerveDrive);

    // TODO: None of this will work if the math is out here and not in execute()

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (hasNote) {
      m_swerveDrive.setTrackingState(TRACKING_STATE.SPEAKER);
      m_readyTimer.start();
      m_shotTimer.stop();
      m_shotTimer.reset();
    }
    if (m_readyTimer.hasElapsed(0.5)) {

      m_ampShooter.setAutoPercentOutput(STATE.SHOOTING.get());

      m_intake.setSpeed(0.5, 0.85);
    }
    if (!hasNote) {
      m_shotTimer.start();
      m_readyTimer.stop();
      m_readyTimer.reset();
    }

    if (!hasNote && m_shotTimer.hasElapsed(m_timeToShoot)) {
      m_swerveDrive.setTrackingState(TRACKING_STATE.NONE);
      m_intake.setSpeed(0.5, 0.85);
      m_ampShooter.setAutoPercentOutput(STATE.AUTOINTAKING.get());
    }

    // else if (hasNote && m_shoottimer.hasElapsed(m_timeToShoot)){
    //   m_intake.setSpeed(-0.5, -0.85);
    // m_ampShooter.setAutoPercentOutput(STATE.REVERSE.get());
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.setTrackingState(TRACKING_STATE.NONE);
    m_ampShooter.setPercentOutput(0);
    m_intake.setSpeed(0, 0);

    m_readyTimer.stop();
    m_readyTimer.reset();
    m_shotTimer.stop();
    m_shotTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
