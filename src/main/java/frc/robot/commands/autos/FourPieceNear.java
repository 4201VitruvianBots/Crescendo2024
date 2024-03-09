// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
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
    var IntakeFactory = new AutoFactory.IntakeFactory(intake, ampShooter);
    var ShooteFactory = new AutoFactory.ShootFactory(intake, ampShooter, shooter);

    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.MAX.get());

    addCommands(
        AutoFactory.createAutoInit(swerveDrive, pathFactory, fieldSim),
        pathFactory.getNextPathCommand().alongWith(flywheelCommandContinuous),
        ShooteFactory.generateShootCommand(),
        new WaitCommand(0.75),
        pathFactory.getNextPathCommand().alongWith(IntakeFactory.generateIntakeCommand()),
        ShooteFactory.generateShootCommand(),
        new WaitCommand(0.75),
        pathFactory.getNextPathCommand().alongWith(IntakeFactory.generateIntakeCommand()),
        ShooteFactory.generateShootCommand(),
        new WaitCommand(0.75),
        pathFactory.getNextPathCommand().alongWith(IntakeFactory.generateIntakeCommand()),
        ShooteFactory.generateShootCommand());

    // commandList.get(4).andThen(() -> swerveDrive.setControl(stopRequest));
  }
}
