package frc.robot.constants;

public final class CAN {
  public static final String rioCanbus = "rio";

  public static final int CANdle = 8;
  public static final int pigeon = 9;

  public static final int frontLeftCanCoder = 10;
  public static final int frontRightCanCoder = 11;
  public static final int backLeftCanCoder = 12;
  public static final int backRightCanCoder = 13;

  public static final int frontLeftDriveMotor = 20;
  public static final int frontLeftTurnMotor = 21;
  public static final int frontRightDriveMotor = 22;
  public static final int frontRightTurnMotor = 23;
  public static final int backLeftDriveMotor = 24;
  public static final int backLeftTurnMotor = 25;
  public static final int backRightDriveMotor = 26;
  public static final int backRightTurnMotor = 27;

  public static final int intakeMotor1 = 30;
  public static final int intakeMotor2 = 31;
  public static final int uptakeMotor = 35;

  public enum INTAKE_STATE {
    // Units are in Percent Output
    NONE(0),
    INTAKING(0.6);

    private final double value;

    INTAKE_STATE(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public enum UPTAKE_STATE {
    // Units are in Percent Output
    NONE(0),
    UPTAKING(0.6);

    private final double value;

    UPTAKE_STATE(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public static final int flywheel1 = 41;
  public static final int flywheel2 = 42;

  public static final int ampShooter = 51;
  public static final int armMotor = 52;
}
