// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.SetRobotPose;
import frc.robot.commands.intake.AutoRunAmpTake;
import frc.robot.commands.shooter.AutoScore;
import frc.robot.constants.AMP;
import frc.robot.constants.INTAKE;
import frc.robot.constants.INTAKE.STATE;
import frc.robot.constants.SHOOTER.RPM_SETPOINT;
import frc.robot.constants.SHOOTER.WAIT;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceFar extends SequentialCommandGroup {
  /** Creates a new ThreePieceFar. */
  public ThreePieceFar(
      CommandSwerveDrivetrain swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      AmpShooter ampShooter,
      Shooter shooter) {
    String[] pathFiles = {
      "3Piece2Pt1", "3Piece2Pt2", "3Piece2Pt3", "3Piece2Pt4",
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

    var turnToShoot = swerveDrive.turnInPlace(Rotation2d.fromDegrees(-45), Controls::isRedAlliance);
    var turnToPath = swerveDrive.turnInPlace(Rotation2d.fromDegrees(0), Controls::isRedAlliance);
    var point = new SwerveRequest.PointWheelsAt();
    var stopRequest = new SwerveRequest.ApplyChassisSpeeds();

    var RunIntake =
        new AutoRunAmpTake(
            intake,
            ampShooter,
            STATE.FRONT_ROLLER_INTAKING.get(),
            STATE.BACK_ROLLER_INTAKING.get(),
            frc.robot.constants.AMP.STATE.INTAKING.get());

    var shootCommand =
        new AutoScore(
            shooter,
            ampShooter,
            intake,
            AMP.STATE.INTAKING.get(),
            RPM_SETPOINT.SPEAKER.get(),
            INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
            WAIT.SHOOTING.get(),
            3);

    addCommands(
        new PlotAutoPath(fieldSim, "", pathsList),
        // new InstantCommand(()-> swerveDrive.resetGyro(0), swerveDrive),
        new SetRobotPose(swerveDrive, pathsList.get(0).getPreviewStartingHolonomicPose()),
        new InstantCommand(
                () -> swerveDrive.applyRequest(() -> point.withModuleDirection(new Rotation2d())),
                swerveDrive)
            .alongWith(new WaitCommand(1)),
        commandList.get(0),
        commandList.get(1),
        commandList.get(2),
        commandList.get(3).andThen(() -> swerveDrive.setControl(stopRequest)));
  }
}
