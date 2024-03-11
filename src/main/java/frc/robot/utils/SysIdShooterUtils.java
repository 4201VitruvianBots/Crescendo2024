package frc.robot.utils;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.Shooter;

public class SysIdShooterUtils {
  public static SysIdRoutine createShooterRoutines(Shooter shooter) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(10).per(Units.Seconds.of(1)),
            Volts.of(65),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new Mechanism(
            (Measure<Voltage> volts) -> {
              var currentOutput = volts.in(Volts);
              shooter.setFocCurrentOutput(currentOutput);
            },
            null,
            shooter));
  }
}
