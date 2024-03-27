// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

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
public class OneWaitAuto extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public OneWaitAuto(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      AmpShooter ampShooter,
      Shooter shooter) {
    String[] pathFiles = {"SimpleAuto1", "SimpleAuto2"};
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);
    var intakeFactory = new AutoFactory.IntakeFactory(intake, ampShooter);
    var shooterFactory = new AutoFactory.ShootFactory(intake, ampShooter, shooter);

    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.MAX.get());

    var Wait = new WaitCommand(8);
    addCommands(
        pathFactory.createAutoInit(),
        Wait,
        pathFactory.getNextPathCommand().alongWith(flywheelCommandContinuous),
        new AutoSetTrackingState(swerveDrive, intake, VISION.TRACKING_STATE.SPEAKER),
        shooterFactory.generateShootCommand().withTimeout(0.75),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                intakeFactory.generateIntakeCommand(),
                new AutoSetTrackingState(swerveDrive, intake, VISION.TRACKING_STATE.NOTE)));
  }
}
