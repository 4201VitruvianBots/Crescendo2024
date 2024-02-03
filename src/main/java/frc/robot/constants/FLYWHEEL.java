package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class FLYWHEEL {
  public static double gearRatio = 1.0;
  public static final int kSlotIdx = 0;

  public static final double kP = 0.6;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kG = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

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
