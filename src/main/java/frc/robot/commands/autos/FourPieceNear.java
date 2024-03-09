// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.intake.AutoRunAmpTake;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.AMPSHOOTER;
import frc.robot.constants.INTAKE;
import frc.robot.constants.INTAKE.STATE;
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

    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.MAX.get());

    var shootCommand =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            INTAKE.STATE.NONE.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.INTAKING.get());

    var shootCommand2 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            INTAKE.STATE.NONE.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.INTAKING.get());
    var shootCommand3 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            INTAKE.STATE.NONE.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.INTAKING.get());

    var shootCommand4 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            INTAKE.STATE.NONE.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.INTAKING.get());

    var shootCommand5 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            INTAKE.STATE.NONE.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.INTAKING.get());
    var shootCommand6 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            INTAKE.STATE.NONE.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.INTAKING.get());
    var RunIntake =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            STATE.FRONT_ROLLER_INTAKING.get(),
            STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.NONE.get());
    var RunIntake4 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            STATE.FRONT_ROLLER_INTAKING.get(),
            STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.NONE.get());

    var RunIntake5 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            STATE.FRONT_ROLLER_INTAKING.get(),
            STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.NONE.get());

    var RunIntake2 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            STATE.FRONT_ROLLER_INTAKING.get(),
            STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.NONE.get());

    var RunIntake3 =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            STATE.FRONT_ROLLER_INTAKING.get(),
            STATE.BACK_ROLLER_INTAKING.get(),
            AMPSHOOTER.STATE.NONE.get());

    addCommands(
        AutoFactory.createAutoInit(swerveDrive, pathFactory, fieldSim),
        pathFactory.getNextPathCommand().alongWith(flywheelCommandContinuous),
        shootCommand,
        new WaitCommand(0.75),
        commandList.get(1).alongWith(RunIntake),
        shootCommand2,
        new WaitCommand(0.75),
        commandList.get(2).alongWith(RunIntake2),
        shootCommand3,
        new WaitCommand(0.75),
        commandList.get(3).alongWith(RunIntake3),
        shootCommand4);

    // commandList.get(4).andThen(() -> swerveDrive.setControl(stopRequest));
  }
}
