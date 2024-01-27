// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FLYWHEEL.FLYWHEEL_STATE;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import java.util.function.DoubleSupplier;

public class ShootNStrafe extends Command {
  Shooter m_shooter;
  FLYWHEEL_STATE m_state;
  SwerveDrive m_swerveDrive;
  private double m_rpm;

  private final DoubleSupplier m_throttleInput, m_strafeInput, m_rotationInput;
  private double displacementX = 5.548 - m_swerveDrive.getOdometry().getEstimatedPosition().getX();
  private double displacementY = m_swerveDrive.getOdometry().getEstimatedPosition().getY();
  private double VelocityY = 0;
  private double VelocityX = 0;
  private double VelocityShoot = 0;
  double m_headingOffset =
      Math.asin(
          Math.abs(
              (displacementY * VelocityX - displacementX * VelocityY)
                  / ((Math.sqrt(Math.pow(displacementX, 2) + Math.pow(displacementY, 2))))
                  * VelocityShoot));

  public ShootNStrafe(
      SwerveDrive swerveDrive,
      Vision vision,
      DoubleSupplier throttleInput,
      DoubleSupplier strafeInput,
      DoubleSupplier rotationInput,
      double RPM) {
    m_swerveDrive = swerveDrive;
    m_throttleInput = throttleInput;
    m_strafeInput = strafeInput;
    m_rotationInput = rotationInput;
    m_rpm = RPM;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_shooter.setRPM(m_rpm);

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

    m_swerveDrive.drive(throttle, strafe, rotation + m_headingOffset, true, false);
  }

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_shooter.setRPM(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
