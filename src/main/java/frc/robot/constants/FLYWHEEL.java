package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class FLYWHEEL {
  public static double gearRatio = 1.0;

  public static final double kDistanceFromIntake = Units.inchesToMeters(19);

  public enum WAIT {
    WAIT_FOR_FLYWHEEL_SETPOINT(0.8),
    WAIT_FOR_AMP_SCORE(0.8);

    private final double value;

    WAIT(double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public enum FLYWHEEL_STATE {
    NONE(0),
    SPEAKER(1);

    private final double value;

    FLYWHEEL_STATE(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
