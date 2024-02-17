package frc.robot.utils;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.Shooter;

public class SysIdShooterUtils {
  private static SysIdRoutine shooterRoutine;

  public static SysIdRoutine createShooterRoutines(Shooter shooter) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null, (state) -> SignalLogger.writeString("state", state.toString())),
        new Mechanism(
            (Measure<Voltage> volts) -> {
              shooter.setFocCurrentOutput(volts.in(Volts));
            },
            null,
            shooter));
  }
}
