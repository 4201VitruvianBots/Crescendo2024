// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.AMPSHOOTER.STATE;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShootNStrafe extends Command {

  private final AmpShooter m_ampShooter;
  private final Intake m_intake;
  private final Shooter m_shooter;

  private final Timer m_readyTimer = new Timer();
  private final Timer m_shotTimer = new Timer();

  private final double m_timeToShoot = 0.75;

  public AutoShootNStrafe(AmpShooter ampShooter, Intake intake, Shooter shooter) {
m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_intake = intake;


    // TODO: None of this will work if the math is out here and not in execute()

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_readyTimer.stop();
    m_readyTimer.reset();
    m_shotTimer.stop();
    m_shotTimer.reset();
  }

  @Override
  public void execute() {
    if (m_shooter.getRpmMaster() > 7500 || m_shooter.getRpmFollower() > 7500 ) {
      m_ampShooter.setPercentOutput(STATE.SHOOTING.get());
    }
    // if (m_intake.checkEitherIntakeSensorActive()) {
    //   m_readyTimer.start();
    //   m_shotTimer.stop();
    //   m_shotTimer.reset();
    // }
    // if (m_readyTimer.hasElapsed(0.5)) {

    //   m_ampShooter.setAutoPercentOutput(STATE.SHOOTING.get());

    //   m_intake.setSpeed(0.5, 0.85);
    // }
    // if (!m_intake.checkEitherIntakeSensorActive()) {
    //   m_shotTimer.start();
    //   m_readyTimer.stop();
    //   m_readyTimer.reset();
    // }

    // if (!m_intake.checkEitherIntakeSensorActive() && m_shotTimer.hasElapsed(m_timeToShoot)) {
    //   m_intake.setSpeed(0.5, 0.85);
    //   m_ampShooter.setAutoPercentOutput(STATE.AUTOINTAKING.get());
    // }

    // else if (hasNote && m_shoottimer.hasElapsed(m_timeToShoot)){
    //   m_intake.setSpeed(-0.5, -0.85);
    // m_ampShooter.setAutoPercentOutput(STATE.REVERSE.get());
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
