// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.AutoSetTrackingState;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.VISION;
import frc.robot.constants.VISION.TRACKING_STATE;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FivePiece extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public FivePiece(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      AmpShooter ampShooter,
      Shooter shooter) {
    String[] pathFiles = {
      "5Piecept1", "5Piecept2", "5Piecept3", "5Piecept4", "5Piecept5", "5Piecept6", "5Piecept7"
    };
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);
    var intakeFactory = new AutoFactory.IntakeFactory(intake, ampShooter);
    var shooterFactory = new AutoFactory.ShootFactory(intake, ampShooter, shooter);
    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.AUTO_RPM.get());

    // TODO: After testing vision, make shoot command check for angle to speaker with a tolerance
    // value
    // TODO: Need to think about how long to aim before shooting?
    addCommands(
        pathFactory.createAutoInit(),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                flywheelCommandContinuous,
                new AutoSetTrackingState(swerveDrive, VISION.TRACKING_STATE.SPEAKER)), // path1
        shooterFactory.generateShootCommand().withTimeout(2.5),
        pathFactory.getNextPathCommand().alongWith(intakeFactory.generateIntakeCommand()), // path2
        shooterFactory.generateShootCommand().withTimeout(2.25),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                intakeFactory.generateIntakeCommand(),
                new AutoSetTrackingState(swerveDrive, TRACKING_STATE.NONE)),
        pathFactory
            .getNextPathCommand()
            .alongWith(new AutoSetTrackingState(swerveDrive, TRACKING_STATE.SPEAKER)), // path4
        shooterFactory.generateShootCommand().withTimeout(2.0),
        pathFactory.getNextPathCommand().alongWith(intakeFactory.generateIntakeCommand()),
        shooterFactory.generateShootCommand().withTimeout(2.0),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                intakeFactory.generateIntakeCommand(),
                new AutoSetTrackingState(swerveDrive, VISION.TRACKING_STATE.SPEAKER)));
  }
}
