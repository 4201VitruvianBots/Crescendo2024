package frc.robot.utils;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.ModuleMap.MODULE_POSITION;

public class SysidUtils {
  private static SysIdRoutine[] swerveDriveRoutine = new SysIdRoutine[4];
  private static SysIdRoutine[] swerveTurnRoutine = new SysIdRoutine[4];

  public static void createSwerveDriveRoutines(SwerveDrive swerveDrive) {
    for (var position : MODULE_POSITION.values()) {
      var module = swerveDrive.getSwerveModule(position);
      swerveDriveRoutine[position.ordinal()] =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  Volts.of(4),
                  null,
                  (state) -> SignalLogger.writeString("state", state.toString())),
              new Mechanism(
                  (Measure<Voltage> volts) -> {
                    module.setDriveSysidVoltage(volts.in(Volts));
                  },
                  null,
                  module));
    }
  }

  public static void createSwerveTurnRoutines(SwerveDrive swerveDrive) {
    for (var position : MODULE_POSITION.values()) {
      var module = swerveDrive.getSwerveModule(position);
      swerveDriveRoutine[position.ordinal()] =
          new SysIdRoutine(
              new SysIdRoutine.Config(
                  null,
                  Volts.of(4),
                  null,
                  (state) -> SignalLogger.writeString("state", state.toString())),
              new Mechanism(
                  (Measure<Voltage> volts) -> {
                    module.setTurnSysidVoltage(volts.in(Volts));
                  },
                  null,
                  module));
    }
  }

  public static SysIdRoutine[] getSwerveModuleDriveRoutines() {
    return swerveDriveRoutine;
  }
}
