// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FLYWHEEL.FLYWHEEL_STATE;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.RobotContainer;
import frc.robot.constants.SWERVE;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

public class ShootNStrafe extends Command {
  Shooter m_shooter;
  FLYWHEEL_STATE m_state;
  CommandSwerveDrivetrain m_swerveDrive;
  private double m_percentOutput;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
  private double PoseX = m_swerveDrive.getState().Pose.getX();
  private double PoseY = m_swerveDrive.getState().Pose.getY();
  private double shootangle = m_shooter.shootangle(PoseX,PoseY);
  
  private double displacementX = loremipsum * Math.sin(shootangle);
  private double displacementY = loremipsum * Math.cos(shootangle);
  
  private double VelocityY =
    m_swerveDrive.getChassisSpeed().omegaRadiansPerSecond * m_swerveDrive.getState().Pose.getRotation().getSin();
  private double VelocityX =
      m_swerveDrive.getChassisSpeed().omegaRadiansPerSecond * m_swerveDrive.getState().Pose.getRotation().getCos();
  private double VelocityShoot = 1.2; //TODO: Change after testing

  double m_headingOffset =
      Math.asin(
          Math.abs(
              (displacementY * VelocityX - displacementX * VelocityY)
                  / ((Math.sqrt(Math.pow(displacementX, 2) + Math.pow(displacementY, 2)))
                  * VelocityShoot)));

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(SWERVE.DRIVE.kMaxSpeedMetersPerSecond * 0.1)
          .withRotationalDeadband(
              SWERVE.DRIVE.kMaxRotationRadiansPerSecond * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

  public ShootNStrafe(
      CommandSwerveDrivetrain swerveDrive,
      Vision vision,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput,
      double percentOutput) {
    m_swerveDrive = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_percentOutput = percentOutput;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_shooter.setPercentOutput(m_percentOutput);

    double throttle =
        MathUtil.applyDeadband(Math.abs(m_throttleInput.getAsDouble()), 0.05)
            * Math.signum(m_throttleInput.getAsDouble());
    double strafe =
        MathUtil.applyDeadband(Math.abs(m_strafeInput.getAsDouble()), 0.05)
            * Math.signum(m_strafeInput.getAsDouble());
    double rotation =
        MathUtil.applyDeadband(Math.abs(m_rotationInput.getAsDouble()), 0.05)
            * Math.signum(m_rotationInput.getAsDouble());

    //    if (DriverStation.isFMSAttached()
    //        && Controls.getAllianceColor() == DriverStation.Alliance.Red) {
    //      throttle *= -1;
    //      strafe *= -1;
    //    }
    if (inZone && (timePassed == false)){
    m_swerveDrive.setControl(
      drive.withVelocityX((throttle) * DRIVE.kMaxSpeedMetersPerSecond)
            .withVelocityY((strafe)* DRIVE.kMaxSpeedMetersPerSecond)
            .withRotationalRate(((rotation) * DRIVE.kMaxRotationRadiansPerSecond)+ m_headingOffset));
    }

    // m_swerveDrive.applyRequest(() -> RobotContainer.drive
    // .withVelocityX((-Axis1) * DRIVE.kMaxSpeedMetersPerSecond)//forward
    // .withVelocityY((-Axis0)* DRIVE.kMaxSpeedMetersPerSecond)//strafe
    // .withRotationalRate((-Axis2) * DRIVE.kMaxRotationRadiansPerSecond));

   
   
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
