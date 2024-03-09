// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.AutoSetTrackingState;
import frc.robot.commands.drive.SetTrackingState;
import frc.robot.constants.VISION;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeTestVision extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public IntakeTestVision(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      AmpShooter ampShooter,
      Shooter shooter) {
    String[] pathFiles = {"DriveForward"};
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);
    var intakeFactory = new AutoFactory.IntakeFactory(intake, ampShooter);
    var shooterFactory = new AutoFactory.ShootFactory(intake, ampShooter, shooter);

    addCommands(
        AutoFactory.createAutoInit(swerveDrive, pathFactory, fieldSim),
        pathFactory
            .getNextPathCommand()
            .alongWith(
                intakeFactory.generateIntakeCommand(),
                new AutoSetTrackingState(swerveDrive, VISION.TRACKING_STATE.NOTE)));
  }
}
