// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.commands.shooter.AutoShootNStrafe;
import frc.robot.constants.AMPSHOOTER.STATE;
import frc.robot.constants.INTAKE;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Telemetry;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSOTMTest extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public AutoSOTMTest(
      CommandSwerveDrivetrain swerveDrive,
      Telemetry telemetry,
      FieldSim fieldSim,
      Intake intake,
      AmpShooter ampShooter,
      Shooter shooter) {
    String[] pathFiles = {"2PieceNearPart1"};
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);

    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

    var RunIntake =
        new AutoRunIntake(
            intake,
            INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get());

    var shootCommand =
        new AutoShootNStrafe(
            swerveDrive,
            telemetry,
            ampShooter,
            shooter,
            intake,
            INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            STATE.INTAKING.get(),
            1,
            5);

    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.MAX.get());

    addCommands(
        AutoFactory.createAutoInit(swerveDrive, pathFactory, fieldSim),
        flywheelCommandContinuous,
        new WaitCommand(2),
        pathFactory.getNextPathCommand().alongWith(shootCommand));
  }
}
