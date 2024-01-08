package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Objects;

public class BASE {
  public static String robotName = "";

  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public enum ROBOT {
    // Robot Serial Numbers
    GRIDLOCK("0306ce62"),
    BOBOT(""),
    SIM("");

    private final String value;

    ROBOT(final String value) {
      this.value = value;
    }

    public String getSerial() {
      return value;
    }

    public String getName() {
<<<<<<< Updated upstream
      return name();
=======
      return this.name();
>>>>>>> Stashed changes
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

  public static void initGridlock() {}

  public static void initBobot() {}

  public static void initSim() {}

  public static void initConstants() {
    if (RobotController.getSerialNumber().equals(ROBOT.GRIDLOCK.getSerial())) {
      initGridlock();
    } else if (RobotController.getSerialNumber().equals(ROBOT.BOBOT.getSerial())) {
      initBobot();
    } else if (RobotController.getSerialNumber().equals(ROBOT.SIM.getSerial())) {
      initSim();
    } else {
      System.out.printf(
          "WARN: Robot Serial Not Recognized! Current roboRIO Serial: %s\n",
          RobotController.getSerialNumber());
    }
  }
}
