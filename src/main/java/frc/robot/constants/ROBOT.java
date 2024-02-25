package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;

public class ROBOT {
  public static String robotName = "";
  public static final boolean disableLogging = false;
  public static final boolean disableVisualization = false;
  public static final boolean useSysID = false;
  public static final boolean useReplayLogs = false;

  public static final double drivebaseWidth = Units.inchesToMeters(26.0);
  public static final double drivebaseLength = Units.inchesToMeters(27.5);
  public static final double robotHeight = Units.inchesToMeters(27.5);

  public static final String ANSI_RESET = "\u001B[0m";
  public static final String ANSI_BLACK = "\u001B[30m";
  public static final String ANSI_RED = "\u001B[31m";
  public static final String ANSI_GREEN = "\u001B[32m";
  public static final String ANSI_YELLOW = "\u001B[33m";
  public static final String ANSI_BLUE = "\u001B[34m";
  public static final String ANSI_PURPLE = "\u001B[35m";
  public static final String ANSI_CYAN = "\u001B[36m";
  public static final String ANSI_WHITE = "\u001B[37m";
  
  public enum CONTROL_MODE {
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public enum ROBOT_ID {
    // Robot Serial Numbers
    FORTE("FORTRE"),
    ALPHABOT("030cbcf0"),
    GRIDLOCK("0306ce62"),
    BOBOT("030e6a97"),
    SIM("");

    public String value = "";

    ROBOT_ID(final String value) {
      this.value = value;
    }

    public String getSerial() {
      return value;
    }

    public String getName() {
      return name();
    }
  }

  public enum SETPOINT {
    // Units are in Radians
    STOWED(Units.degreesToRadians(0.0));

    public double value;

    SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public static void initForte() {}

  public static void initAlphaBot() {}

  public static void initGridlock() {
    SWERVE.DRIVE.kFrontLeftEncoderOffset = -0.035888671875;
    SWERVE.DRIVE.kFrontRightEncoderOffset = 0.04296875;
    SWERVE.DRIVE.kBackLeftEncoderOffset = 0.483642578125;
    SWERVE.DRIVE.kBackRightEncoderOffset = 0.414306640625;

    SWERVE.DRIVE.kInvertLeftDrive = false;
    SWERVE.DRIVE.kInvertRightDrive = true;

    SWERVE.DRIVE.kTrackWidth = Units.inchesToMeters(24);
    CAN.drivebaseCanbus = CAN.rioCanbus;
  }

  public static void initBobot() {
    SWERVE.DRIVE.kFrontLeftEncoderOffset = Units.degreesToRotations(95.27328);
    SWERVE.DRIVE.kFrontRightEncoderOffset = Units.degreesToRotations(34.18956);
    SWERVE.DRIVE.kBackLeftEncoderOffset = Units.degreesToRotations(77.51952);
    SWERVE.DRIVE.kBackRightEncoderOffset = Units.degreesToRotations(330.55668);

    SWERVE.DRIVE.kInvertLeftDrive = false;
    SWERVE.DRIVE.kInvertRightDrive = true;
  }

  public static void initSim() {
    SWERVE.DRIVE.kFrontLeftEncoderOffset = 0;
    SWERVE.DRIVE.kFrontRightEncoderOffset = 0;
    SWERVE.DRIVE.kBackLeftEncoderOffset = 0;
    SWERVE.DRIVE.kBackRightEncoderOffset = 0;

    // Different gear ratios seem to break SimpleJointedArmSim
    //    ARM.gearRatio = 1.0;
  }

  public static void initConstants() {
    var alert = new Alert("Initializing Robot Constants...", AlertType.INFO);

    if (RobotController.getSerialNumber().equals(ROBOT_ID.FORTE.getSerial())) {
      alert.setText("Setting Robot Constants for FORTE");
      initForte();
    } else if (RobotController.getSerialNumber().equals(ROBOT_ID.ALPHABOT.getSerial())) {
      alert.setText("Setting Robot Constants for ALPHABOT");
      initAlphaBot();
    } else if (RobotController.getSerialNumber().equals(ROBOT_ID.GRIDLOCK.getSerial())) {
      alert.setText("Setting Robot Constants for Gridlock");
      initGridlock();
    } else if (RobotController.getSerialNumber().equals(ROBOT_ID.BOBOT.getSerial())) {
      alert.setText("Setting Robot Constants for Bobot");
      initBobot();
    } else if (RobotController.getSerialNumber().equals(ROBOT_ID.SIM.getSerial())) {
      alert.setText("Setting Robot Constants for Sim");
      initSim();
    } else {
      alert =
          new Alert(
              "WARN: Robot Serial Not Recognized! Current roboRIO Serial: "
                  + RobotController.getSerialNumber(),
              AlertType.WARNING);
    }
    alert.set(true);
  }
}
