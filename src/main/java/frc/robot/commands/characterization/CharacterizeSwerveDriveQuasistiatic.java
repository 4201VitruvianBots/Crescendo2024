// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.ModuleMap.MODULE_POSITION;
import frc.robot.utils.SysidUtils;

public class CharacterizeSwerveDriveQuasistiatic extends SequentialCommandGroup {
  /** Creates a new CharacterizeSwerveDriveQuasistiatic. */
  public CharacterizeSwerveDriveQuasistiatic(
      SwerveDrive swerveDrive, SysIdRoutine.Direction direction) {
    var routines = SysidUtils.getswervemodueldrivRoutines();
    Command[] Characteriztioncommand = new Command[4];
    for (int i = 0; i < 4; i++) {
      Characteriztioncommand[i] = routines[i].quasistatic(direction);
    }
    Command[] initCommands = new Command[4];
    for (var position : MODULE_POSITION.values()) {
      var module = swerveDrive.getSwerveModule(position);
      initCommands[position.ordinal()] =
          new InstantCommand((() -> module.InitalizeDrivecharacterization()));
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
        initCommands[0],
        initCommands[1],
        initCommands[2],
        initCommands[3],
        new ParallelCommandGroup(
            Characteriztioncommand[0],
            Characteriztioncommand[1],
            Characteriztioncommand[2],
            Characteriztioncommand[3]));
  }
}
