// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class INTAKE {
  public static final double intakeLength = Units.inchesToMeters(4);
  public static final double gearRatio = 1.0 / 1.0;
  public static final DCMotor intake1Gearbox = DCMotor.getKrakenX60(1);
  public static final DCMotor intake2Gearbox = DCMotor.getKrakenX60(1);
  public static final double Inertia = 0.001;
  /* A higher P value means you will put more effort into correcting the measured error,
  but it means you can overshoot your target and then the response will look like an oscillating graph. */
  public static final double kP = 0.085;

  /* I value is generally used to correct steady-state error
  (e.g. your goal is 100 but you are at 99, so the sum of error
  over time will let it correct for that final error). */
  public static final double kI = 0.0;

  /* D is generally used to 'predict' the next output using the slope of the error,
  so it is usually used with P to get a fast, but accurate response. */
  public static final double kD = 13.0;

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
