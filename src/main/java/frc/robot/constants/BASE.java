package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class BASE {
  public static String robotName = "";

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public static final class CONSTANTS {
    public static final int kFalconSensorUnitsPerRotation = 2048;
    public static final int kCANCoderSensorUnitsPerRotation = 4096;
  }

  public enum SETPOINT {
    // Units are in Radians

    STOWED(Units.degreesToRadians(98.0)),
    INTAKING_LOW_CUBE(Units.degreesToRadians(-13.5)),
    SCORE_LOW_REVERSE(Units.degreesToRadians(-14.0)),
    SCORE_MID_CUBE(Units.degreesToRadians(127.0)), // Temporary value
    SCORE_HIGH_CUBE(Units.degreesToRadians(147.0)),
    INTAKING_EXTENDED_CUBE(SCORE_HIGH_CUBE.get());

    public double getWristSetpointRadians() {
      return value;
    }

    private final double value;

    SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
