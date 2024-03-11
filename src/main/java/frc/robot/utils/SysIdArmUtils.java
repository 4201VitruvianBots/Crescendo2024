// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.Arm;

/** Add your docs here. */
public class SysIdArmUtils {
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private static final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private static final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private static final MutableMeasure<Velocity<Angle>> m_velocity =
      mutable(RotationsPerSecond.of(0));

  public static SysIdRoutine createArmRoutines(Arm arm) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.2).per(Units.Seconds.of(1)), Volts.of(1.5), null),
        new Mechanism(
            (Measure<Voltage> volts) -> {
              var currentOutput = volts.in(Volts);
              arm.getMotor().setVoltage(currentOutput);
            },
            log -> {
              log.motor("armMotor")
                  .voltage(m_appliedVoltage.mut_replace(arm.getInputVoltage(), Volts))
                  .angularPosition(m_angle.mut_replace(arm.getCurrentRotation(), Rotations))
                  .angularVelocity(
                      m_velocity.mut_replace(arm.getRotationalVelocity(), RotationsPerSecond));
            },
            arm));
  }
}
