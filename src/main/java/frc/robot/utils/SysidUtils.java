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
  private static SysIdRoutine swerveDriveRoutine;
  private static SysIdRoutine swerveTurnRoutine;

  public static void createSwerveDriveRoutines(SwerveDrive swerveDrive) {
    swerveDriveRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(
                (Measure<Voltage> volts) -> {
                  for (var position : MODULE_POSITION.values()) {
                    swerveDrive.getSwerveModule(position).setDriveSysidVoltage(volts.in(Volts));
                  }
                },
                null,
                swerveDrive));
  }

  public static void createSwerveTurnRoutines(SwerveDrive swerveDrive) {
    var module = swerveDrive.getSwerveModule(MODULE_POSITION.FRONT_LEFT);
    swerveTurnRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, (state) -> SignalLogger.writeString("state", state.toString())),
            new Mechanism(
                (Measure<Voltage> volts) -> module.setTurnSysidVoltage(volts.in(Volts)),
                null,
                module));
  }

  public static SysIdRoutine getSwerveDriveRoutine() {
    return swerveDriveRoutine;
  }

  public static SysIdRoutine getSwerveTurnRoutine() {
    return swerveTurnRoutine;
  }
}
