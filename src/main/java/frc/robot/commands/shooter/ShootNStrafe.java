// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Telemetry;
import java.util.function.DoubleSupplier;

public class ShootNStrafe extends Command {

  private final CommandSwerveDrivetrain m_swerveDrive;
  private final Shooter m_shooter;
  // private final AmpShooter m_ampShooter;
  // private final Intake m_intake;
  private final Telemetry m_telemetry;

  // private double m_AmpPercentOutput;
  private double m_RPMOutput;
  // private final double m_FrontIntakePercentOutput;
  // private final double m_BackIntakeAmpPercentOutput;

  private final Timer m_timer = new Timer();
  private final Timer m_reversetimer = new Timer();
  private final Timer m_shoottimer = new Timer();

  private final double reverseTimerThreshold = 0.25;
  private final double m_timeToShoot = 0.75;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  private double allowableError = 300;

  public final int hehe = 69; // Mano's work

  private final PIDController m_turnController =
      new PIDController(SWERVE.DRIVE.kP_Theta, SWERVE.DRIVE.kI_Theta, SWERVE.DRIVE.kD_Theta);
  private Translation2d m_target = new Translation2d();

  private double m_targetx;
  private double m_targety;

  //   private final SwerveRequest.FieldCentric drive =
  //       new SwerveRequest.FieldCentric()
  //           .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
  //           .withRotationalDeadband(
  //               SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
  //           .withDriveRequestType(
  //               SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

  public ShootNStrafe(
      CommandSwerveDrivetrain swerveDrive,
      Telemetry telemetry,
      // AmpShooter ampShooter,
      Shooter shooter,
      // Intake intake,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput,
      // double AmpPercentOutput,
      double RPMOutput
      // ,
      // double FrontIntakeAmpPercentOutput,
      // double BackIntakeAmpPercentOutput
      ) {

    m_swerveDrive = swerveDrive;
    m_telemetry = telemetry;
    // m_intake = intake;
    m_shooter = shooter;
    // m_ampShooter = ampShooter;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    // m_AmpPercentOutput = AmpPercentOutput;
    // m_FrontIntakePercentOutput = FrontIntakeAmpPercentOutput;
    // m_BackIntakeAmpPercentOutput = BackIntakeAmpPercentOutput;
    m_RPMOutput = RPMOutput;

    addRequirements(m_swerveDrive);

    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
    m_turnController.setTolerance(Units.degreesToRadians(2.0));

    // TODO: None of this will work if the math is out here and not in execute()

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_timer.stop();
    // m_timer.reset();
    // m_reversetimer.stop();
    // m_reversetimer.reset();
    // m_shoottimer.stop();
    // m_shoottimer.reset();
    // m_turnController.reset();
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

    Pose2d robotPose = m_swerveDrive.getState().Pose;
    // double shootAngle = m_shooter.getShootAngle(robotPose);

    double effectiveDistance = Units.metersToFeet(1.5); // meters
    Translation2d currentPose = m_swerveDrive.getState().Pose.getTranslation();

    double displacementY = m_swerveDrive.getState().Pose.getY() - m_targety;
    double displacementX = m_swerveDrive.getState().Pose.getX() - m_targetx;

    Translation2d robotToGoal = m_target.minus(m_swerveDrive.getState().Pose.getTranslation());

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

    /*public static double getOffsetAngleDeg(double effectiveDistance) {
        //Pose2d goalRel = getGoalRelPose();
        Pose2d pose = new Pose2d(robotPose.getX() - Constants.goalLocation.getX(), robotPose.getY() - Constants.goalLocation.getY(), robotPose.getRotation());
        return Units.radiansToDegrees(
          Math.asin(airTime *
            (getGlobalMecVy() * pose.getX() + getGlobalMecVx() * pose.getY())
            / (getActualDistance() * effectiveDistance)
          )
        );
    */

    double getOffsetAngleDeg =
        Math.asin((VelocityY * PositionX + VelocityX * PositionY) / (newDist * effectiveDistance));

    // double m_headingOffset =
    //     Math.asin(
    //         displacementY * VelocityX
    //             - displacementX
    //                 * VelocityY
    //                 / ((Math.sqrt(Math.pow(displacementX, 2) + Math.pow(displacementY, 2)))
    //                     * VelocityShoot));
    // System.out.println(m_headingOffset * 180 / Math.PI);

    // System.out.println(getOffsetAngleDeg);

    final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(
                SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
            .withDriveRequestType(
                SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    // double rotation =
    //     (m_swerveDrive.getState().Pose.getRotation().getRadians() - shootAngle) // Jax's code
    //         + m_headingOffset; // Jadon's code

    // all of the logic for angle is above this Comment

    var targetDelta = (m_swerveDrive.getState().Pose.getTranslation().minus(m_target).getAngle());

    m_shooter.setRPMOutput(m_RPMOutput);

    m_swerveDrive.setControl(
        drive
            .withVelocityX((-m_throttleInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withVelocityY((-m_strafeInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withRotationalRate(
                m_turnController.calculate(
                    m_swerveDrive.getState().Pose.getRotation().getRadians(),
                    targetDelta.getRadians() + getOffsetAngleDeg)));
    // if (inZone
    //     && m_shooter.getRpmMaster() >= (m_RPMOutput - allowableError)
    //     && m_shooter.getRpmFollower() >= (m_RPMOutput - allowableError)) {

    //   m_ampShooter.setPercentOutput(-m_AmpPercentOutput);

    //   m_reversetimer.start();

    //   if (m_reversetimer.hasElapsed(reverseTimerThreshold)) {
    //     m_ampShooter.setPercentOutput(m_AmpPercentOutput);
    //     m_intake.setSpeed(m_FrontIntakePercentOutput, m_BackIntakeAmpPercentOutput);
    //     m_shoottimer.start();
    //   }
  }

  // Math.atan2(
  //       (SHOOTER.NoteVelocity * Math.sin(getShootAngle(robotPose)) - RobotVelocityY),
  //       (SHOOTER.NoteVelocity * Math.cos(getShootAngle(robotPose)) - RobotVelocityX));

  // }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setPercentOutput(0);
    // m_ampShooter.setPercentOutput(0);
    // m_intake.setSpeed(0, 0);
    // m_timer.stop();
    // m_timer.reset();
    // m_reversetimer.stop();
    // m_reversetimer.reset();
    // m_shoottimer.stop();
    // m_shoottimer.reset();
    final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(
                SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
            .withDriveRequestType(
                SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    m_swerveDrive.setControl(
        drive
            .withVelocityX((m_throttleInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withVelocityY((m_strafeInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withRotationalRate((m_rotationInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond));
    // m_timer.stop();
    // m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_shoottimer.hasElapsed(m_timeToShoot);
    return false;
  }
}
