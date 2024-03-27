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
public class FourPieceNear extends SequentialCommandGroup {
  /** Creates a new DriveStraightTest. */
  public FourPieceNear(
      CommandSwerveDrivetrain swerveDrive,
      Shooter shooter,
      AmpShooter ampShooter,
      Intake intake,
      FieldSim fieldSim) {

    String[] pathFiles = {
      "FourPiecePt1", "FourPiecePt2", "FourPiecePt3", "FourPiecePt4",
      //   "FourPiecePt4.1","FourPiecePt5"
    };
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);
    var intakeFactory = new AutoFactory.IntakeFactory(intake, ampShooter);
    var shooterFactory = new AutoFactory.ShootFactory(intake, ampShooter, shooter);

    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.AUTO_RPM.get());

    // TODO: After testing vision, make shoot command check for angle to speaker with a tolerance
    // value
    // TODO: Need to think about how long to aim before shooting?
    addCommands(
        pathFactory.createAutoInit(),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                flywheelCommandContinuous, // path 1
                new AutoSetTrackingState(swerveDrive, intake, VISION.TRACKING_STATE.SPEAKER)),
        shooterFactory.generateShootCommand().withTimeout(2.5),
        pathFactory // path 2
            .getNextPathCommand()
            .alongWith(intakeFactory.generateIntakeCommand()),
        shooterFactory.generateShootCommand().withTimeout(1.5),
        pathFactory.getNextPathCommand().alongWith(intakeFactory.generateIntakeCommand()),
        shooterFactory.generateShootCommand().withTimeout(1.5),
        pathFactory.getNextPathCommand().alongWith(intakeFactory.generateIntakeCommand()),
        shooterFactory.generateShootCommand().withTimeout(1.5));

    // commandList.get(4).andThen(() -> swerveDrive.setControl(stopRequest));
  }
}
