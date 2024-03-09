// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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


  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;

  public final int hehe = 69; // Mano's work

  private final PIDController m_turnController =
      new PIDController(SWERVE.DRIVE.kTeleP_Theta, SWERVE.DRIVE.kTeleI_Theta, SWERVE.DRIVE.kTeleD_Theta);
  private Translation2d m_target = new Translation2d();

  public ShootNStrafe(
      CommandSwerveDrivetrain swerveDrive,
      
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput) {

    m_swerveDrive = swerveDrive;
 
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;

    addRequirements(m_swerveDrive);

    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
    m_turnController.setTolerance(Units.degreesToRadians(2.0));


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if (Controls.isRedAlliance()) {
      m_target = FIELD.redSpeaker;

    
    } else {
      m_target = FIELD.blueSpeaker;
    }

    var targetDelta = (m_swerveDrive.getState().Pose.getTranslation().minus(m_target).getAngle());
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

    Translation2d movingGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

    Translation2d toMovingGoal = movingGoalLocation.minus(currentPose);

    double newDist = toMovingGoal.getDistance(new Translation2d());

    double airtime = newDist / VelocityShoot;

    // double ED = Units.metersToFeet(1.5);

   double getOffsetAngleDeg =
        Math.asin((airtime * (VelocityY * PositionX + VelocityX * PositionY)) / (newDist));

    SmartDashboard.putNumber("SOTM/ AngleOffset", getOffsetAngleDeg);
    SmartDashboard.putNumber("SOTM/ Angle", ((targetDelta.getDegrees() + 360) % 360));
    SmartDashboard.putNumber("SOTM/ Velocity", VelocityY);

    final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    // all of the logic for angle is above this Comment

    if (Controls.getAllianceColor() == DriverStation.Alliance.Red) {
    m_swerveDrive.setControl(
        drive
            .withVelocityX((-m_throttleInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withVelocityY((-m_strafeInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withRotationalRate(
                m_turnController.calculate(
                    m_swerveDrive.getState().Pose.getRotation().getRadians(),
                    (targetDelta.getRadians()
                        + getOffsetAngleDeg))));

    }

    else if (Controls.getAllianceColor() == DriverStation.Alliance.Blue) {

m_swerveDrive.setControl(
        drive
            .withVelocityX((m_throttleInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withVelocityY((m_strafeInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withRotationalRate(
                m_turnController.calculate(
                    m_swerveDrive.getState().Pose.getRotation().getRadians(),
                    (targetDelta.getRadians()
                        + getOffsetAngleDeg))));
    }
  }

  @Override
  public void end(boolean interrupted) {

    // m_shooter.setPercentOutput(0);

    final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
            .withRotationalDeadband(SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1)
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    m_swerveDrive.setControl(
        drive
            .withVelocityX((m_throttleInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withVelocityY((m_strafeInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond)
            .withRotationalRate((m_rotationInput.getAsDouble()) * DRIVE.kMaxSpeedMetersPerSecond));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
