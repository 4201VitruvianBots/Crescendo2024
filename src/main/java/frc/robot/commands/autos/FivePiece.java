// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.shooter.AutoSetRPMSetpoint;
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
    var pathFactory = new AutoFactory.PathFactory(swerveDrive, pathFiles);
    var IntakeFactory = new AutoFactory.IntakeFactory(intake, ampShooter);
    var ShooteFactory = new AutoFactory.ShootFactory(intake, ampShooter, shooter);
    var flywheelCommandContinuous = new AutoSetRPMSetpoint(shooter, RPM_SETPOINT.MAX.get());

    addCommands(
        AutoFactory.createAutoInit(swerveDrive, pathFactory, fieldSim),
        pathFactory.getNextPathCommand().alongWith(flywheelCommandContinuous),
        ShooteFactory.generateShootCommand().withTimeout(0.75),
        pathFactory.getNextPathCommand().alongWith(IntakeFactory.generateIntakeCommand()),
        ShooteFactory.generateShootCommand().withTimeout(0.75),
        pathFactory.getNextPathCommand().alongWith(IntakeFactory.generateIntakeCommand()),
        pathFactory.getNextPathCommand(),
        ShooteFactory.generateShootCommand().withTimeout(0.75),
        pathFactory.getNextPathCommand().alongWith(IntakeFactory.generateIntakeCommand()),
        ShooteFactory.generateShootCommand().withTimeout(0.75),
         pathFactory.getNextPathCommand().alongWith(IntakeFactory.generateIntakeCommand()),
        pathFactory.getNextPathCommand(),
        ShooteFactory.generateShootCommand().withTimeout(0.75));
  }
}
