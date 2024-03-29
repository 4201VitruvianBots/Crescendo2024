// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.intake.AutoRunAmpTake;
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.AMPSHOOTER;
import frc.robot.constants.INTAKE;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPiece extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public TwoPiece(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      AmpShooter ampShooter,
      Shooter shooter) {
    String[] pathFiles = {"2PieceNearPart1", "2PieceNearPart2"};
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);

    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

    var RunIntake =
        new AutoRunIntake(
            intake,
            INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get());

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

    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.AUTO_RPM.get());

    addCommands(
        pathFactory.createAutoInit(),
        pathFactory.getNextPathCommand().alongWith(flywheelCommandContinuous),
        shootCommand,
        new WaitCommand(1),
        pathFactory.getNextPathCommand().alongWith(RunIntake),
        shootCommand2);
  }
}
