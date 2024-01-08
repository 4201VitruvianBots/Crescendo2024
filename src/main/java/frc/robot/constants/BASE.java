package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class BASE {
  public static String robotName = "";

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public enum ROBOT {
    // Robot Serial Numbers
    GRIDLOCK("0306ce62");

    private final String value;

    ROBOT(final String value) {
      this.value = value;
    }

    public String getSerial() {
      return value;
    }

    public String getName() {
      return this.ordinal();
    }
  }

  public enum SETPOINT {
    // Units are in Radians
    STOWED(Units.degreesToRadians(0.0));

    private final double value;

    SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }
}
