package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.SetRobotPose;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;

public class AutoFactory {
  public static class PathFactory {
    private ArrayList<PathPlannerPath> pathList = new ArrayList<>();
    private ArrayList<Command> commandList = new ArrayList<>();
    private int pathCounter;

    public PathFactory(CommandSwerveDrivetrain swerveDrive, String[] pathNames) {
      for (var filename : pathNames) {
        var path = PathPlannerPath.fromPathFile(filename);
        var command =
            TrajectoryUtils.generatePPHolonomicCommand(
                swerveDrive, path, path.getGlobalConstraints().getMaxVelocityMps());
        pathList.add(path);
        commandList.add(command);
      }
    }

    public ArrayList<PathPlannerPath> getPathList() {
      return pathList;
    }

    public Command getNextPathCommand() {
      if (pathCounter > pathList.size())
        throw new ArrayIndexOutOfBoundsException(
            "Called getNextPathCommand too many times for this set of paths!");
      else return commandList.get(pathCounter++);
    }

    public Pose2d getStartingPose() {
      return pathList.get(0).getPreviewStartingHolonomicPose();
    }
  }

  public static SequentialCommandGroup createAutoInit(
      CommandSwerveDrivetrain swerveDrive, AutoFactory.PathFactory pathFactory) {
    return new AutoInit(swerveDrive, pathFactory);
  }

  public static SequentialCommandGroup createAutoInit(
      CommandSwerveDrivetrain swerveDrive, AutoFactory.PathFactory pathFactory, FieldSim fieldSim) {
    return new AutoInit(swerveDrive, pathFactory, fieldSim);
  }

  private static class AutoInit extends SequentialCommandGroup {
    static SwerveRequest.PointWheelsAt swervePointRequest = new SwerveRequest.PointWheelsAt();

    public AutoInit(CommandSwerveDrivetrain swerveDrive, AutoFactory.PathFactory pathFactory) {
      addCommands(
          new SetRobotPose(swerveDrive, pathFactory.getStartingPose()),
          new InstantCommand(
              () -> swerveDrive.applyRequest(() -> swervePointRequest), swerveDrive));
    }

    public AutoInit(
        CommandSwerveDrivetrain swerveDrive,
        AutoFactory.PathFactory pathFactory,
        FieldSim fieldSim) {
      addCommands(
          new PlotAutoPath(fieldSim, "", pathFactory.getPathList()),
          new SetRobotPose(swerveDrive, pathFactory.getStartingPose()),
          new InstantCommand(
              () -> swerveDrive.applyRequest(() -> swervePointRequest), swerveDrive));
    }
  }

  public static class ShootFactory {
    public ShootFactory(Intake intake, AmpShooter ampShooter, Shooter shooter) {}
  }

  public static class IntakeFactory {
    public IntakeFactory(Intake intake, AmpShooter ampShooter) {}
  }


}
