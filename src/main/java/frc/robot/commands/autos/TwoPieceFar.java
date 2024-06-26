// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.AutoSetTrackingState;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.VISION;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPieceFar extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public TwoPieceFar(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      AmpShooter ampShooter,
      Shooter shooter) {
    String[] pathFiles = {"TwoPieceFarPt1", "TwoPieceFarPt2", "TwoPieceFarPt3", "TwoPieceFarPt4"};
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles, fieldSim);
    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();
    var intakeFactory = new AutoFactory.IntakeFactory(intake, ampShooter);
    var shooterFactory = new AutoFactory.ShootFactory(intake, ampShooter, shooter);
    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.AUTO_RPM.get());

    addCommands(
        pathFactory.createAutoInit(),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                flywheelCommandContinuous,
                new AutoSetTrackingState(swerveDrive, VISION.TRACKING_STATE.SPEAKER)), // path1
        shooterFactory.generateShootCommand().withTimeout(2.5),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                intakeFactory.generateIntakeCommand(),
                new AutoSetTrackingState(swerveDrive, VISION.TRACKING_STATE.NONE)),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                new AutoSetTrackingState(swerveDrive, VISION.TRACKING_STATE.SPEAKER)), // path1
        shooterFactory.generateShootCommand().withTimeout(2),
        pathFactory.getNextPathCommand().andThen(() -> swerveDrive.setControl(stopRequest)));
  }
}
