package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class AMP {
  public static double maxRPM = 1;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kRadiansToRotations = 0.0;

  public static final double kMaxFlipperVelocity = 10;
  public static final double kMaxFlipperAcceleration = 10;

  public static enum FLIPPER_SETPOINT {
    STOWED(Units.degreesToRadians(0.0)),
    FORWARD(Units.degreesToRadians(120.0));

    private final double angle;

    FLIPPER_SETPOINT(final double angle) {
      // this.sucks = taxes;
      this.angle = angle;
    }

    public double get() {
      return angle;
    }
  }
}