// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.SetRobotPose;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.SimConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourPieceNear extends SequentialCommandGroup {
  /** Creates a new DriveStraightTest. */
  public FourPieceNear(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim) {
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
        commandList.get(3),
        commandList.get(4).andThen(() -> swerveDrive.setControl(stopRequest)));
  }
}
