// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.SetRobotPose;
import frc.robot.commands.shooter.AutoScore;
import frc.robot.constants.AMP;
import frc.robot.constants.INTAKE;
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
public class FourPieceNear extends SequentialCommandGroup {
  /** Creates a new DriveStraightTest. */
  public FourPieceNear(
      CommandSwerveDrivetrain swerveDrive,
      Shooter shooter,
      AmpShooter ampShooter,
      Intake intake,
      FieldSim fieldSim) {
    String[] pathFiles = {
      "FourPiecePt1", "FourPiecePt2", "FourPiecePt3", "FourPiecePt4", "FourPiecePt5",
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
        new AutoScore(
            shooter,
            ampShooter,
            intake,
            AMP.STATE.INTAKING.get(),
            RPM_SETPOINT.SPEAKER.get(),
            INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
            INTAKE.STATE.BACK_ROLLER_INTAKING.get());

    addCommands(
        new PlotAutoPath(fieldSim, "", pathsList),
        // new InstantCommand(()-> swerveDrive.resetGyro(0), swerveDrive),
        new SetRobotPose(swerveDrive, pathsList.get(0).getPreviewStartingHolonomicPose()),
        commandList.get(0),
        shootCommand.withTimeout(1),
        commandList.get(1),
        commandList.get(2),
        commandList.get(3),
        commandList.get(4).andThen(() -> swerveDrive.setControl(stopRequest)));
  }
}
