// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;
import java.util.logging.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class ShootNStrafe extends Command {

  private final CommandSwerveDrivetrain m_swerveDrive;
  private final Shooter m_shooter;
  private final AmpShooter m_ampShooter;
  private final Intake m_intake;

  private double m_AmpPercentOutput;
  private double m_RPMOutput;
  private final double m_FrontIntakePercentOutput;
  private final double m_BackIntakeAmpPercentOutput;

  private final Timer m_timer = new Timer();
  private final Timer m_reversetimer = new Timer();
  private final Timer m_shoottimer = new Timer();
  Pose2d m_pose2d;
  private final double reverseTimerThreshold = 0.25;
  private final double m_timeToShoot = 0.75;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  private double allowableError = 300;
  private boolean inZone = true; // change to toggle when we are in our shooting zone

  public final int hehe = 69; // Mano's work

  //   private final SwerveRequest.FieldCentric drive =
  //       new SwerveRequest.FieldCentric()
  //           .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
  //           .withRotationalDeadband(
  //               SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
  //           .withDriveRequestType(
  //               SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

  public ShootNStrafe(
      CommandSwerveDrivetrain swerveDrive,
      AmpShooter ampShooter,
      Shooter shooter,
      Intake intake,
      Pose2d pose2d,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput,
      double AmpPercentOutput,
      double RPMOutput,
      double FrontIntakeAmpPercentOutput,
      double BackIntakeAmpPercentOutput) {

    m_swerveDrive = swerveDrive;
    m_intake = intake;
    m_shooter = shooter;
    m_pose2d = pose2d;
    m_ampShooter = ampShooter;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_AmpPercentOutput = AmpPercentOutput;
    m_FrontIntakePercentOutput = FrontIntakeAmpPercentOutput;
    m_BackIntakeAmpPercentOutput = BackIntakeAmpPercentOutput;
    m_RPMOutput = RPMOutput;

    addRequirements(m_swerveDrive);

    // TODO: None of this will work if the math is out here and not in execute()

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.stop();
    m_timer.reset();
    m_reversetimer.stop();
    m_reversetimer.reset();
    m_shoottimer.stop();
    m_shoottimer.reset();
  }

  @Override
  public void execute() {
     m_swerveDrive.setDefaultCommand(
        m_swerveDrive.applyFieldCentricDrive(
            () ->
                new ChassisSpeeds(
                    m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
                    m_strafeInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
                    m_rotationInput.getAsDouble() * DRIVE.kMaxRotationRadiansPerSecond)));
  
    Pose2d robotPose = m_swerveDrive.getState().Pose;
    double shootAngle = m_shooter.getShootAngle(robotPose);

    double displacementX = m_pose2d.getX() * Math.sin(shootAngle);

    double displacementY = m_pose2d.getY() * Math.cos(shootAngle);

    double VelocityY =
        m_swerveDrive.getChassisSpeed().omegaRadiansPerSecond
            * m_swerveDrive.getState().Pose.getRotation().getSin();
    double VelocityX =
        m_swerveDrive.getChassisSpeed().omegaRadiansPerSecond
            * m_swerveDrive.getState().Pose.getRotation().getCos();
    double VelocityShoot = Units.feetToMeters(116.937059884); // TODO: Change after testing

    double m_headingOffset =
        Math.asin(
            displacementY * VelocityX
                - displacementX
                    * VelocityY
                    / ((Math.sqrt(Math.pow(displacementX, 2) + Math.pow(displacementY, 2)))
                        * VelocityShoot));

        final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
          .withRotationalDeadband(
              SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric
                

    // TODO: @jax when are we using this vs var rotation?

    double rotation =
        (m_swerveDrive.getState().Pose.getRotation().getRadians() - shootAngle) // Jax's code
            + m_headingOffset; // Jadon's code

    // all of the logic for angle is above this Comment

    m_shooter.setRPMOutput(m_RPMOutput);
    
    m_swerveDrive.setControl(
      drive
          .withVelocityX((m_throttleInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
          .withVelocityY((m_strafeInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
          .withRotationalRate(rotation));
 
        // m_swerveDrive.applyFieldCentricDrive(
        //     () ->
        //         new ChassisSpeeds(
        //             m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
        //             m_strafeInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
        //             0));
   
  //  System.out.println(rotation);
   
   System.out.println(shootAngle);

    if (inZone
        && m_shooter.getRpmMaster() >= (m_RPMOutput - allowableError)
        && m_shooter.getRpmFollower() >= (m_RPMOutput - allowableError)) {

      m_ampShooter.setPercentOutput(-m_AmpPercentOutput);

      m_reversetimer.start();

      if (m_reversetimer.hasElapsed(reverseTimerThreshold)) {
        m_ampShooter.setPercentOutput(m_AmpPercentOutput);
        m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);
        m_shoottimer.start();
      }
    }
  }

  //   m_ampShooter.setPercentOutput(-m_AmpPercentOutput);

  //   m_reversetimer.start();

  //   if (m_reversetimer.hasElapsed(reverseTimerThreshold)) {
  //     m_ampShooter.setPercentOutput(m_AmpPercentOutput);
  //     m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);
  //     m_shoottimer.start();

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setPercentOutput(0);
    m_ampShooter.setPercentOutput(0);
    m_intake.setSpeed(0, 0);
    m_timer.stop();
    m_timer.reset();
    m_reversetimer.stop();
    m_reversetimer.reset();
    m_shoottimer.stop();
    m_shoottimer.reset();

    m_swerveDrive.setDefaultCommand(
        m_swerveDrive.applyFieldCentricDrive(
            () ->
                new ChassisSpeeds(
                    m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
                    m_strafeInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
                    m_rotationInput.getAsDouble() * DRIVE.kMaxRotationRadiansPerSecond)));
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shoottimer.hasElapsed(m_timeToShoot);
  }
}
