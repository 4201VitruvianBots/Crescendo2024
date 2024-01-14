// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.ModuleMap.MODULE_POSITION;
import frc.robot.utils.SysidUtils;

public class SwerveDriveDynamic extends SequentialCommandGroup {
  /** Creates a new SwerveDriveDynamic. */
  public SwerveDriveDynamic(SwerveDrive swerveDrive, SysIdRoutine.Direction direction) {
    var routines = SysidUtils.getSwerveModuleDriveRoutines();

    Command[] initCommands = new Command[routines.length];
    for (var position : MODULE_POSITION.values()) {
      var module = swerveDrive.getSwerveModule(position);
      initCommands[position.ordinal()] = new InstantCommand((module::initDriveSysid));
    }

    Command[] sysidCommands = new Command[routines.length];
    for (int i = 0; i < routines.length; i++) {
      sysidCommands[i] = routines[i].dynamic(direction);
    }

    SwerveModuleState[] states =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
        };

    addCommands(
        new InstantCommand(() -> swerveDrive.setSwerveModuleStates(states, false)),
        new WaitCommand(1),
        initCommands[0],
        initCommands[1],
        initCommands[2],
        initCommands[3],
        new ParallelCommandGroup(
                sysidCommands[0], sysidCommands[1], sysidCommands[2], sysidCommands[3])
            .withTimeout(10)
            .andThen(() -> swerveDrive.setChassisSpeed(new ChassisSpeeds())));
  }
}
