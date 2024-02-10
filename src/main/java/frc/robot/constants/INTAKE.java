// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class INTAKE {
  public static final double intakeLength = Units.inchesToMeters(4);

  public enum INTAKE_STATE {
    // Units are in Percent Output
    NONE(0),
    INTAKING(-0.6),
    INTAKING1(-0.85),
    INTAKINGSLOW(0.2);
    private final double value;

    INTAKE_STATE(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
