// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;

public class ShootNStrafe extends Command {

  private final CommandSwerveDrivetrain m_swerveDrive;
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;

  private final double timerThreshold = 0.5;
  private boolean CorrectRange = true;

  private final Timer m_timer = new Timer();
  private boolean timerStart = false;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  private RPM_SETPOINT mstate;
  private double m_RPMOutput;
  private double RPMThreshold = m_RPMOutput;
  private double ChangeThisValue;

  public final int hehe = 69; // Mano's work

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
          .withRotationalDeadband(
              SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

  public ShootNStrafe(
      CommandSwerveDrivetrain swerveDrive,
      AmpShooter ampShooter,
      Shooter shooter,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput,
      double RPMOutput) {

    m_swerveDrive = swerveDrive;

    m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_RPMOutput = RPMOutput;

    addRequirements(m_swerveDrive);

    // TODO: None of this will work if the math is out here and not in execute()

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timerStart = false;
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_swerveDrive.getState().Pose;
    double shootAngle = m_shooter.getShootAngle(robotPose);

    double displacementX = ChangeThisValue * Math.sin(shootAngle); // TODO: Change this

    double displacementY = ChangeThisValue * Math.cos(shootAngle);

    double VelocityY =
        m_swerveDrive.getChassisSpeed().omegaRadiansPerSecond
            * m_swerveDrive.getState().Pose.getRotation().getSin();
    double VelocityX =
        m_swerveDrive.getChassisSpeed().omegaRadiansPerSecond
            * m_swerveDrive.getState().Pose.getRotation().getCos();
    double VelocityShoot = 1.2; // TODO: Change after testing

    double m_headingOffset =
        Math.asin(
            Math.abs(
                (displacementY * VelocityX - displacementX * VelocityY)
                    / ((Math.sqrt(Math.pow(displacementX, 2) + Math.pow(displacementY, 2)))
                        * VelocityShoot)));

    m_shooter.setRpmOutput(RPMThreshold);

    double throttle =
        MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.05)
            * Math.signum(m_throttleInput.getAsDouble());
    double strafe =
        MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.05)
            * Math.signum(m_strafeInput.getAsDouble());
    double rotation =
        MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.05)
            * Math.signum(m_rotationInput.getAsDouble());

    if (CorrectRange
        && m_shooter.getRpmMaster() >= RPMThreshold
        && m_shooter.getRpmFollower() >= RPMThreshold) {

      drive.withVelocityX(VelocityX).withVelocityY(VelocityY).withRotationalRate(m_headingOffset);
      m_timer.reset();
      m_timer.start();
      timerStart = true;

      if (timerStart && m_timer.hasElapsed(timerThreshold)) {
        isFinished();
      }
    } else {
      m_swerveDrive.setControl(
          drive
              .withVelocityX((throttle) * DRIVE.kMaxSpeedMetersPerSecond)
              .withVelocityY((strafe) * DRIVE.kMaxSpeedMetersPerSecond)
              .withRotationalRate(m_headingOffset));
      m_timer.reset();
      m_timer.stop();
      timerStart = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setPercentOutput(0);
    m_ampShooter.setPercentOutput(0);

    double throttle =
        MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.05)
            * Math.signum(m_throttleInput.getAsDouble());
    double strafe =
        MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.05)
            * Math.signum(m_strafeInput.getAsDouble());
    double rotation =
        MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.05)
            * Math.signum(m_rotationInput.getAsDouble());

    m_swerveDrive.setDefaultCommand(
        m_swerveDrive.applyFieldCentricDrive(
            () ->
                new ChassisSpeeds(
                    throttle * DRIVE.kMaxSpeedMetersPerSecond,
                    strafe * DRIVE.kMaxSpeedMetersPerSecond,
                    rotation * DRIVE.kMaxRotationRadiansPerSecond)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
