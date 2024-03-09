// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.SetRobotPose;
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
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;

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
    ArrayList<PathPlannerPath> pathsList = new ArrayList<>();
    ArrayList<Command> commandList = new ArrayList<>();

    for (var filename : pathFiles) {
      var path = PathPlannerPath.fromPathFile(filename);
      var command =
          TrajectoryUtils.generatePPHolonomicCommand(
              swerveDrive, path, path.getGlobalConstraints().getMaxVelocityMps());
      pathsList.add(path);
      commandList.add(command);
    }

    var point = new SwerveRequest.PointWheelsAt();
    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

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
    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.MAX.get());

    addCommands(
        new PlotAutoPath(fieldSim, "", pathsList),
        new SetRobotPose(swerveDrive, pathsList.get(0).getPreviewStartingHolonomicPose()),
        commandList.get(0).alongWith(flywheelCommandContinuous),
        shootCommand,
        new WaitCommand(0.75),
        commandList.get(1).alongWith(RunIntake),
        shootCommand2,
        new WaitCommand(0.75),
        commandList.get(2).alongWith(RunIntake2),
        commandList.get(3),
        shootCommand3,
        new WaitCommand(0.75),
        commandList.get(4).alongWith(RunIntake3),
        shootCommand4,
        new WaitCommand(0.75),
        commandList.get(5).alongWith(RunIntake4),
        commandList.get(6),
        shootCommand5);
  }
}
