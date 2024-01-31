package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class UPTAKE {
  public static final double uptakeLength = Units.inchesToMeters(18);

  public enum UPTAKE_STATE {
    DOWNTAKE(-0.8),
    NONE(0),
    UPTAKE(0.8),
    UPTAKING(0.6);

    private final double value;

    UPTAKE_STATE(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
