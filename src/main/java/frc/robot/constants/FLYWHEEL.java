package frc.robot.constants;

public final class FLYWHEEL {
  public static double gearRatio = 1.0;

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
