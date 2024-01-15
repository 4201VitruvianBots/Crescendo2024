/* Copyright (c) FIRST and other WPILib contributors.
   Open Source Software; you can modify and/or share it under the terms of
   the WPILib BSD license file in the root directory of this project. */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import frc.robot.constants.CAN;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  
  private final CANdle m_candle = new CANdle(CAN.CANdle);
  private int red = 0;
  private int green = 0; // setting all LED colors to none: there is no color when robot activates
  private int blue = 0;
  // TODO: Use enum for robot state
  private boolean setSolid;
  private Animation m_toAnimate = null;

  public LED(Controls controls) {
    m_candle.configFactoryDefault(); 
    // sets up LED strip
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true; // sets lights of when the LEDs are activated
    configAll.disableWhenLOS = false; // disables LEDs when there is no signal for control
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar =
        0.5; // 1 is highest we can go we don't want to blind everyone at the event
    configAll.vBatOutputMode = VBatOutputMode.Modulated; // Modulate
    m_candle.configAllSettings(configAll, 100);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_2_Startup, 255);
    m_candle.setStatusFramePeriod(
        CANdleStatusFrame.CANdleStatusFrame_Status_3_FirmwareApiStatus, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_4_ControlTelem, 255);
    m_candle.setStatusFramePeriod(
        CANdleStatusFrame.CANdleStatusFrame_Status_5_PixelPulseTrain, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_6_BottomPixels, 255);
    m_candle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_7_TopPixels, 255);
  }
  // will create LED patterns
  public void setPattern(Color8Bit color, int white, double speed, ANIMATION_TYPE toChange) {
    int red = color.red;
    int green = color.green;
    int blue = color.blue;
    switch (toChange) {
      case ColorFlow: // stripe of color flowing through the LED strip
        m_toAnimate =
            new ColorFlowAnimation(red, green, blue, white, speed, LED.LEDcount, Direction.Forward);
        break;
      case Fire: // red and orange LEDs flaming up and down the LED strip
        m_toAnimate = new FireAnimation(0.5, 0.7, LED.LEDcount, 0.7, 0.5);
        break;
      case Larson: // a line bouncing back and forth with its width determined by size
        m_toAnimate =
            new LarsonAnimation(red, green, blue, white, speed, LED.LEDcount, BounceMode.Front, 7);
        break;
      case Rainbow: // neon cat type beat
        m_toAnimate = new RainbowAnimation(1, speed, LED.LEDcount);
        break;
      case RgbFade: // cycling between red, greed, and blue
        m_toAnimate = new RgbFadeAnimation(1, speed, LED.LEDcount);
        break;
      case SingleFade: // slowly turn all LEDs from solid color to off
        m_toAnimate = new SingleFadeAnimation(red, green, blue, white, speed, LED.LEDcount);
        break;
      case Strobe: // switching between solid color and full off at high speed
        m_toAnimate = new StrobeAnimation(red, green, blue, white, speed, LED.LEDcount);
        break;
      case Twinkle: // random LEDs turning on and off with certain color
        m_toAnimate =
            new TwinkleAnimation(
                red, green, blue, white, speed, LED.LEDcount, TwinklePercent.Percent6);
        break;
      case TwinkleOff: // twinkle in reverse
        m_toAnimate =
            new TwinkleOffAnimation(
                red, green, blue, white, speed, LED.LEDcount, TwinkleOffPercent.Percent100);
        break;
      case Solid:
        this.red = red;
        this.green = green;
        this.blue = blue;
        m_toAnimate = null;
        break;
      default:
        //        System.out.println("Incorrect animation type provided to changeAnimation()
        // method");
        break;
    }
  }
  // will set LEDs a coordinated color for an action
  public void expressState(SUPERSTRUCTURE_STATE state) {
    if (state != currentRobotState) {
      switch (state) {
        case WRIST_IS_RESET:
          setPattern(LED.pink, 0, 0, ANIMATION_TYPE.Solid); // Flashing Pink
          break;
        case INTAKING:
          setPattern(LED.yellow, 0, 0, ANIMATION_TYPE.Strobe); // Flashing Yellow
          break;
        case SCORE_SPEAKER:
          setPattern(LED.turquoise, 0, 0, ANIMATION_TYPE.Solid); // Solid Turquoise
          break;
        case SCORE_AMP:
          setPattern(LED.blue, 0, 0, ANIMATION_TYPE.Solid); // Solid Blue
          break;
        // dunno how many of these we'll use
        case CLIMBING:
          setPattern(LED.white, 15, 0, ANIMATION_TYPE.Solid); // Solid White
          break;  
        case SCORE_TRAP:
          setPattern(LED.pink, 0, 0, ANIMATION_TYPE.Solid); // Solid Pink
          break;
        case DISABLED:
          setPattern(LED.red, 0, 0, ANIMATION_TYPE.Solid); // Solid Red
          break;
        case ENABLED:
          setPattern(LED.green, 0, 0, ANIMATION_TYPE.Solid); // Solid Green
          break;
        case LOW_BATTERY:
          setPattern(LED.yellow, 0, 1, ANIMATION_TYPE.Strobe); // Flashing Yellow
          break;
        default:
          break;
      }
      currentRobotState = state;
    }
  }
  @Override
  public void periodic() {
    // null indicates that the animation is "Solid"
    if (m_toAnimate == null && !setSolid) {
      setSolid = true;
      m_candle.setLEDs(red, green, blue, 0, 0, LED.LEDcount); // setting all LEDs to color
    } else {
      setSolid = false;
      m_candle.animate(m_toAnimate); // setting the candle animation to m_animation if not null
    }

    if (DriverStation.isDisabled()) {
      if (RobotController.getBatteryVoltage()
          < 10) { // calling battery to let driver know that it is low
        expressState(SUPERSTRUCTURE_STATE.LOW_BATTERY);
      }
    }

    SmartDashboard.putString("LED Mode", currentRobotState.toString());
  }
  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {}
}
