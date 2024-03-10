package frc.robot.commands.autos;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.SetRobotPose;
import frc.robot.commands.intake.AutoAmpIntake;
import frc.robot.commands.intake.AutoRunAmpTakeTwo;
import frc.robot.constants.AMPSHOOTER;
import frc.robot.constants.INTAKE;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.AmpShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TrajectoryUtils;
import java.util.ArrayList;

public class AutoFactory {
  public static class PathFactory {
    CommandSwerveDrivetrain m_swerveDrive;
    FieldSim m_fieldSim;

    private ArrayList<PathPlannerPath> pathList = new ArrayList<>();
    private ArrayList<Command> commandList = new ArrayList<>();
    private int pathCounter;

    public PathFactory(CommandSwerveDrivetrain swerveDrive, String[] pathNames) {
      this(swerveDrive, pathNames, null);
    }

    public PathFactory(CommandSwerveDrivetrain swerveDrive, String[] pathNames, FieldSim fieldSim) {
      m_swerveDrive = swerveDrive;
      m_fieldSim = fieldSim;
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

    public SequentialCommandGroup createAutoInit() {
        return new AutoInit(m_swerveDrive, this, m_fieldSim);
    }

    private class AutoInit extends SequentialCommandGroup {
      static SwerveRequest.PointWheelsAt swervePointRequest = new SwerveRequest.PointWheelsAt();

      public AutoInit(
          CommandSwerveDrivetrain swerveDrive,
          AutoFactory.PathFactory pathFactory,
          FieldSim fieldSim) {
        Command plotAutoPath;
        if (fieldSim != null)
          plotAutoPath = new PlotAutoPath(fieldSim, "", pathFactory.getPathList());
        else
          plotAutoPath = new WaitCommand(0);

        addCommands(
            plotAutoPath,
//            new SetRobotPose(swerveDrive, pathFactory.getStartingPose()),
            new InstantCommand(
                () -> swerveDrive.applyRequest(() -> swervePointRequest), swerveDrive));
      }
    }
  }

  public static class ShootFactory {
    private Intake m_Intake;
    private AmpShooter m_AmpShooter;
    private Shooter m_Shooter;

    public ShootFactory(Intake intake, AmpShooter ampShooter, Shooter shooter) {
      m_Intake = intake;
      m_AmpShooter = ampShooter;
      m_Shooter = shooter;
    }

    public Command generateShootCommand() {
      return new AutoRunAmpTakeTwo(
          m_Intake,
          m_AmpShooter,
          INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
          INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
          AMPSHOOTER.STATE.SHOOTING.get(),
          m_Shooter);
    }
  }

  public static class IntakeFactory {
    private Intake m_Intake;
    private AmpShooter m_AmpShooter;

    public IntakeFactory(Intake intake, AmpShooter ampShooter) {
      m_Intake = intake;
      m_AmpShooter = ampShooter;
    }

    public Command generateIntakeCommand() {
      return new AutoAmpIntake(
          m_Intake,
          INTAKE.STATE.FRONT_ROLLER_INTAKING.get(),
          INTAKE.STATE.BACK_ROLLER_INTAKING.get(),
          m_AmpShooter,
          AMPSHOOTER.STATE.INTAKING.get());
    }
  }
}
