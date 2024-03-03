// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.ArrayList;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.SetRobotPose;
import frc.robot.commands.intake.AutoRunAmpTake;
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
import frc.robot.constants.AMPSHOOTER;
import frc.robot.constants.INTAKE;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.SHOOTER.WAIT;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TrajectoryUtils;

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
    String[] pathFiles = {"SimpleAuto1","SimpleAuto2"};
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);
ArrayList<PathPlannerPath> pathsList = new ArrayList<>();
    ArrayList<Command> commandList = new ArrayList<>();
    var point = new SwerveRequest.PointWheelsAt();
    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();
 
    for (var filename : pathFiles) {
      var path = PathPlannerPath.fromPathFile(filename);
      var command =
          TrajectoryUtils.generatePPHolonomicCommand(
              swerveDrive, path, path.getGlobalConstraints().getMaxVelocityMps());
      pathsList.add(path);
      commandList.add(command);
    }
    var runIntake =
        new AutoRunIntake(
            intake,
            INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get());

    var shootCommand =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            INTAKE.STATE.NONE.get(),
            INTAKE.STATE.NONE.get(),
            AMPSHOOTER.STATE.INTAKING.get());

    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.MAX.get());

    var Wait = new WaitCommand(8);
    addCommands(
     Wait,
        new PlotAutoPath(fieldSim, "", pathsList),
        new SetRobotPose(swerveDrive, pathsList.get(0).getPreviewStartingHolonomicPose()),
        commandList.get(0).alongWith(flywheelCommandContinuous),
        shootCommand,
        commandList.get(1).alongWith(runIntake)
    );
  }
}
