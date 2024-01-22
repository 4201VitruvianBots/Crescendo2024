package frc.robot.commands.autos;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;

public class DriveStraightOld extends SequentialCommandGroup {

  public DriveStraightOld(CommandSwerveDrivetrain swerveDrive, FieldSim fieldSim) {

    double maxVel = Units.feetToMeters(16);
    double maxAccel = Units.feetToMeters(16);
    double maxAngularVel = Math.PI * 2;
    double maxAngularAccel = Math.PI * 2;

    if (RobotBase.isSimulation()) {
      maxVel /= 2.0;
      maxAccel /= 2.0;
      maxAngularVel /= 2.0;
      maxAngularAccel /= 2.0;
    }

    var waypoints = new ArrayList<Pose2d>();
    waypoints.add(new Pose2d(new Translation2d(3, 3), Rotation2d.fromDegrees(0)));
    waypoints.add(new Pose2d(new Translation2d(8, 3), Rotation2d.fromDegrees(0)));

    var points = PathPlannerPath.bezierFromPoses(waypoints);

    var path =
        new PathPlannerPath(
            points,
            new PathConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel),
            new GoalEndState(0, Rotation2d.fromDegrees(0)));

    var swerveCommand =
        TrajectoryUtils.generatePPHolonomicCommand(swerveDrive, path, maxVel, false);

    addCommands(
        new InstantCommand(
            () -> swerveDrive.seedFieldRelative(path.getPreviewStartingHolonomicPose())),
        new PlotAutoPath(fieldSim, "DriveStraight", path),
        new WaitCommand(1),
        swerveCommand.andThen(() -> swerveDrive.setChassisSpeed(new ChassisSpeeds())));
  }
}
