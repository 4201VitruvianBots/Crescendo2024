// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.constants.SWERVE;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class SOTM extends Command {
  private final CommandSwerveDrivetrain m_SwerveDrivetrain;
  private final Vision m_vision;
  private final DoubleSupplier m_throttleInput;
  private final DoubleSupplier m_turnInput;
  private final PIDController m_PidController =
      new PIDController(
          SWERVE.DRIVE.kAutoP_Theta, SWERVE.DRIVE.kAutoI_Theta, SWERVE.DRIVE.kAutoD_Theta);
  Translation2d m_goal = new Translation2d();
  double finalTurn = 0.0;

  /** Creates a new rotateRobotToGoal. */
  public SOTM(
      CommandSwerveDrivetrain commandSwerveDrivetrain,
      Vision vision,
      DoubleSupplier throttleInput,
      DoubleSupplier turnInput) {
    m_SwerveDrivetrain = commandSwerveDrivetrain;
    m_vision = vision;
    m_throttleInput = throttleInput;
    m_turnInput = turnInput;
    m_PidController.setTolerance(Units.degreesToRadians(2));
    m_PidController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_SwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Controls.isBlueAlliance()) {
      m_goal = FIELD.blueSpeaker;
    } else {
      m_goal = FIELD.redSpeaker;
    }

    double PositionY = m_SwerveDrivetrain.getState().Pose.getY();
    double PositionX = m_SwerveDrivetrain.getState().Pose.getX();
    double VelocityY = m_SwerveDrivetrain.getChassisSpeed().vyMetersPerSecond;
    double VelocityX = m_SwerveDrivetrain.getChassisSpeed().vxMetersPerSecond;

    double AccelerationX = m_SwerveDrivetrain.getPigeon2().getAccelerationX().getValueAsDouble();
    double AccelerationY = m_SwerveDrivetrain.getPigeon2().getAccelerationY().getValueAsDouble();

    double VelocityShoot = 11.1; // TODO: Change after testing

    double virtualGoalX = m_goal.getX() - VelocityShoot * (VelocityX + AccelerationX);
    double virtualGoalY = m_goal.getY() - VelocityShoot * (VelocityY + AccelerationY);

    Translation2d movingGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
    Translation2d currentPose = m_SwerveDrivetrain.getState().Pose.getTranslation();
    Translation2d toMovingGoal = movingGoalLocation.minus(currentPose);

    double newDist = toMovingGoal.getDistance(new Translation2d());

    double airtime = newDist / VelocityShoot;

    double ED = Units.metersToFeet(1.5);

    double getOffsetAngleDeg =
        Math.asin(((VelocityY * PositionX + VelocityX * PositionY)) / (newDist * ED));

    // SOTM math above
    var setPoint = m_SwerveDrivetrain.getState().Pose.getTranslation().minus(m_goal);
    var turnRate =
        m_PidController.calculate(
            m_SwerveDrivetrain.getState().Pose.getRotation().getRadians(),
            setPoint.getAngle().getRadians());
    finalTurn =
        -MathUtil.clamp(
            turnRate,
            -SWERVE.DRIVE.kMaxRotationRadiansPerSecond,
            SWERVE.DRIVE.kMaxRotationRadiansPerSecond);
    m_SwerveDrivetrain.setChassisSpeedControl(
        new ChassisSpeeds(
            m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
            m_turnInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond, // strafe input
            finalTurn + getOffsetAngleDeg));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}