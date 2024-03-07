// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Telemetry;
import java.util.Optional;

public class AutoShootNStrafe extends Command {

  private final CommandSwerveDrivetrain m_swerveDrive;
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;
  private final Intake m_intake;
  private final Telemetry m_telemetry;

  private double m_AmpPercentOutput;
  private final double m_FrontIntakePercentOutput;
  private final double m_BackIntakeAmpPercentOutput;

  private final Timer m_timer = new Timer();
  private final Timer m_waitToShootTimer = new Timer();
  private final Timer m_shoottimer = new Timer();

  private final Timer m_timeoutTimer = new Timer();

  private final double reverseTimerThreshold = 0.25;
  private final double m_timeToShoot = 0.75;
  private final double m_waitToShoot;
  private final double m_withTimeout;

  private double allowableError = 300;

  private final PIDController m_turnController =
      new PIDController(SWERVE.DRIVE.kP_Theta, SWERVE.DRIVE.kI_Theta, SWERVE.DRIVE.kD_Theta);
  private Translation2d m_target = new Translation2d();

  private double m_targetx;
  private double m_targety;

  public AutoShootNStrafe(
      CommandSwerveDrivetrain swerveDrive,
      Telemetry telemetry,
      AmpShooter ampShooter,
      Shooter shooter,
      Intake intake,
      double FrontIntakeAmpPercentOutput,
      double BackIntakeAmpPercentOutput,
      double AmpPercentOutput,
      double WaitToShoot,
      double withTimeout) {

    m_swerveDrive = swerveDrive;
    m_telemetry = telemetry;
    m_intake = intake;
    m_shooter = shooter;
    m_ampShooter = ampShooter;
    m_AmpPercentOutput = AmpPercentOutput;
    m_FrontIntakePercentOutput = FrontIntakeAmpPercentOutput;
    m_BackIntakeAmpPercentOutput = BackIntakeAmpPercentOutput;
    m_waitToShoot = WaitToShoot;
    m_withTimeout = withTimeout;

    addRequirements(m_swerveDrive);

    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
    m_turnController.setTolerance(Units.degreesToRadians(2.0));

    // TODO: None of this will work if the math is out here and not in execute()

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.stop();
    m_timer.reset();
    m_waitToShootTimer.stop();
    m_waitToShootTimer.reset();
    m_shoottimer.stop();
    m_shoottimer.reset();
    m_turnController.reset();
    m_timeoutTimer.stop();
    m_timeoutTimer.reset();
  }

  @Override
  public void execute() {
    if (Controls.isRedAlliance()) {
      m_target = FIELD.redSpeaker;

      m_targetx = FIELD.redSpeaker.getX();
      m_targety = FIELD.redSpeaker.getY();
    } else {
      m_target = FIELD.blueSpeaker;
      m_targetx = FIELD.blueSpeaker.getX();
      m_targety = FIELD.blueSpeaker.getY();
    }

    double effectiveDistance = Units.metersToFeet(1.5); // meters
    Translation2d currentPose = m_swerveDrive.getState().Pose.getTranslation();

    double PositionY = m_swerveDrive.getState().Pose.getY();
    double PositionX = m_swerveDrive.getState().Pose.getX();
    double VelocityY = m_swerveDrive.getChassisSpeed().vyMetersPerSecond;
    double VelocityX = m_swerveDrive.getChassisSpeed().vxMetersPerSecond;

    double AccelerationX = m_swerveDrive.getPigeon2().getAccelerationX().getValueAsDouble();
    double AccelerationY = m_swerveDrive.getPigeon2().getAccelerationY().getValueAsDouble();

    double VelocityShoot = 11.1; // TODO: Change after testing

    double virtualGoalX = m_target.getX() - VelocityShoot * (VelocityX + AccelerationX);
    double virtualGoalY = m_target.getY() - VelocityShoot * (VelocityY + AccelerationY);

    SmartDashboard.putNumber("Goal X", virtualGoalX);
    SmartDashboard.putNumber("Goal Y", virtualGoalY);

    Translation2d movingGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

    Translation2d toMovingGoal = movingGoalLocation.minus(currentPose);

    double newDist = toMovingGoal.getDistance(new Translation2d());

    double getOffsetAngleDeg =
        Math.asin((VelocityY * PositionX + VelocityX * PositionY) / (newDist * effectiveDistance));

    final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(
                SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    var targetDelta = (m_swerveDrive.getState().Pose.getTranslation().minus(m_target).getAngle());

    // all of the logic for angle is above this Comment

    if (m_intake.getSensorInput1() == true || m_intake.getSensorInput2() == true) {

      m_waitToShootTimer.start();

      PPHolonomicDriveController.setRotationTargetOverride(
          () ->
              Optional.of(
                  Rotation2d.fromDegrees(
                      m_turnController.calculate(
                          m_swerveDrive.getState().Pose.getRotation().getRadians(),
                          targetDelta.getRadians() + getOffsetAngleDeg))));

      if (m_waitToShootTimer.hasElapsed(m_waitToShoot)) {
        m_ampShooter.setPercentOutput(m_AmpPercentOutput);
        m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);
        m_shoottimer.start();
      } else {
        m_ampShooter.setPercentOutput(0);
      }
    } else {
      m_ampShooter.setPercentOutput(m_AmpPercentOutput);
      m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setPercentOutput(0);
    m_ampShooter.setPercentOutput(0);
    m_intake.setSpeed(0, 0);
    m_timer.stop();
    m_timer.reset();
    m_waitToShootTimer.stop();
    m_waitToShootTimer.reset();
    m_shoottimer.stop();
    m_shoottimer.reset();
    m_timeoutTimer.stop();
    m_timeoutTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shoottimer.hasElapsed(m_timeToShoot) || m_timeoutTimer.hasElapsed(m_withTimeout);
  }
}
