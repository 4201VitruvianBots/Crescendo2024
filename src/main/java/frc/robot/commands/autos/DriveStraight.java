package frc.robot.commands.autos;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;

/**
 * Runs a command until a condition is met, then interrupts it to run another command. If the first
 * command finishes, this command will end without running the other command.
 */
// Knock knock
// Who's there?
// Interrupting command
// Interrupting comma-
public class DriveStraight extends Command {
  private final SwerveDrive m_swerveDrive;
  private final FieldSim m_fieldSim;
  private FollowPathHolonomic m_ppCommand;

  public DriveStraight(SwerveDrive swerveDrive, FieldSim fieldSim) {
    m_swerveDrive = swerveDrive;
    m_fieldSim = fieldSim;

    addRequirements(m_swerveDrive);
  }

  @Override
  public void initialize() {
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
    var startPoint = new Pose2d(new Translation2d(3, 3), Rotation2d.fromDegrees(0));
    var endPoint = new Pose2d(new Translation2d(8, 3), Rotation2d.fromDegrees(0));
    waypoints.add(startPoint);
    waypoints.add(endPoint);

    var points = PathPlannerPath.bezierFromPoses(waypoints);

    var path =
        new PathPlannerPath(
            points,
            new PathConstraints(maxVel, maxAccel, maxAngularVel, maxAngularAccel),
            new GoalEndState(0, Rotation2d.fromDegrees(0)));

    var trajectory =
        new PathPlannerTrajectory(
            path, new ChassisSpeeds(), path.getPreviewStartingHolonomicPose().getRotation());

    var pathPoints =
        new ArrayList<>(
            trajectory.getStates().stream()
                .map(PathPlannerTrajectory.State::getTargetHolonomicPose)
                .toList());

    m_fieldSim.setPath(pathPoints);
    m_swerveDrive.setOdometry(startPoint);

    m_ppCommand = TrajectoryUtils.generatePPHolonomicCommand(m_swerveDrive, path, maxVel);
    m_ppCommand.initialize();
  }

  @Override
  public void execute() {
    m_ppCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_ppCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_ppCommand.isFinished();
  }
}
