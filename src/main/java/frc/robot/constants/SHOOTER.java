package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class SHOOTER {

  public static final int kSlotIdx = 0;

  public static final double topkP = 4;
  public static final double topkI = 0;
  public static final double topkD = 0;

  public static final double bottomkP = 8;
  public static final double bottomkI = 0;
  public static final double bottomkD = 0;

  public static final double kS = 0.0;
  public static final double kV = 0.0;
  public static final double kA = 0.0;

  public static final double kShooterAngle = Units.degreesToRadians(35);

  public static final double kDistanceFromIntake = Units.inchesToMeters(19);

  public static final double gearRatioBottom = 28.0 / 20.0;
  public static final DCMotor ShooterBottomGearbox = DCMotor.getFalcon500Foc(1);

  public static final DCMotor ShooterTopGearbox = DCMotor.getFalcon500Foc(1);
  public static final double gearRatioTop = 28.0 / 28.0;

  public static final double Inertia = 0.001;

  public enum WAIT {
    WAIT_FOR_FLYWHEEL_SETPOINT(3),
    WAIT_FOR_AMP_SCORE(0.8);

    private final double value;

    WAIT(double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public enum RPM_SETPOINT {
    NONE(0),

    SLOW(600),
    SPEAKER(4201),
    MAX(6000);

    private final double value;

    RPM_SETPOINT(final double value) {
      this.value = value;
    }

    public double get() {
      return value;
    }
  }

  public static final double NoteVelocity = 0; // add note velocity

  public static class SPEAKER {

    public static final double BlueSpeakerTLY = 6.1478414;
    public static final double BlueSpeakerTLX = 0.464947;

    public static final double BlueSpeakerTRY = 5.0232564;
    public static final double BlueSpeakerTRX = 0.464947;

    public static final double BlueSpeakerBLY = 6.1478414;
    public static final double BlueSpeakerBLX = 0;

    public static final double BlueSpeakerBRY = 5.0232564;
    public static final double BlueSpeakerBRX = 0;

    public static final double RedSpeakerTLY = 6.1478414;
    public static final double RedSpeakerTLX = 16.0697672;

    public static final double RedSpeakerTRY = 5.0232564;
    public static final double RedSpeakerTRX = 16.0697672;

    public static final double RedSpeakerBLY = 6.1478414;
    public static final double RedSpeakerBLX = 16.5410642;

    public static final double RedSpeakerBRY = 5.0232564;
    public static final double RedSpeakerBRX = 16.5410642;
  }
}
