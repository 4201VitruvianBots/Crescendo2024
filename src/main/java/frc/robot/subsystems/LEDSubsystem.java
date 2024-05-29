/* Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and/or share it under the terms of
the WPILib BSD license file in the root directory of this project. */

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CAN;
import frc.robot.constants.LED;
import frc.robot.constants.LED.*;
import frc.robot.constants.ROBOT;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem */
  private final CANdle m_candle = new CANdle(CAN.CANdle);

  private Color8Bit m_color = new Color8Bit();
  private int m_red = 0;
  private int m_green = 0; // setting all LED colors to none: there is no color when robot activates
  private int m_blue = 0;
  private int m_white = 0;
  private double m_brightness = 0;
  private double m_speed = 0;
  private int m_start; // where to start at led strip
  private int m_end; // where to end at led strip
  private SUBSYSTEM_STATES currentRobotState = SUBSYSTEM_STATES.DISABLED;
  private boolean setSolid;
  private Animation m_toAnimate = null;

  public LEDSubsystem() {
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

  /**
   * The method to create a singular or multiple LED patterns on the CANdle
   *
   * @param color what color to set on the LEDs (to add or check the list of available colors, see
   *     LED constants)
   * @param white how much white to set on the LEDs
   * @param speed the animation speed of the LEDs
   * @param toChange what animation to set on the LEDs
   * @param start where to start on the LED strip
   * @param end where to end on the LED strip
   */
  public void setPattern(
      Color8Bit color, int white, double speed, ANIMATION_TYPE toChange, int start, int end) {
    m_color = color;
    m_red = color.red;
    m_green = color.green;
    m_blue = color.blue;
    m_white = white;
    m_speed = speed;
    m_start = start;
    switch (toChange) {
      case ColorFlow: // stripe of color flowing through the LED strip
        m_toAnimate =
            new ColorFlowAnimation(
                m_red, m_green, m_blue, m_white, m_speed, m_end, Direction.Forward, m_start);
        break;
      case ColorFlowLong: // stripe of color flowing through the LED strip
        m_toAnimate =
            new ColorFlowAnimation(
                m_red, m_green, m_blue, m_white, m_speed, m_end + 15, Direction.Forward, m_start);
        break;

      case Fire: // red and orange LEDs flaming up and down the LED strip
        m_brightness = 0.5;
        m_speed = 0.7;
        m_toAnimate = new FireAnimation(m_brightness, m_speed, m_end, 0.7, 0.5, false, m_start);
        break;
      case Larson: // a line bouncing back and forth with its width determined by size
        m_toAnimate =
            new LarsonAnimation(
                m_red, m_green, m_blue, m_white, m_speed, m_end, BounceMode.Front, 7, m_start);
        break;
      case Rainbow: // neon cat type beat
        m_brightness = 1;
        m_toAnimate = new RainbowAnimation(m_brightness, m_speed, m_end, true, m_start);
        break;
      case RgbFade: // cycling between red, greed, and blue
        m_brightness = 1;
        m_toAnimate = new RgbFadeAnimation(m_brightness, m_speed, m_end, m_start);
        break;
      case SingleFade: // slowly turn all LEDs from solid color to off
        m_toAnimate =
            new SingleFadeAnimation(m_red, m_green, m_blue, m_white, m_speed, m_end, m_start);
        break;
      case Strobe: // switching between solid color and full off at high speed
        m_toAnimate = new StrobeAnimation(m_red, m_green, m_blue, m_white, m_speed, m_end, m_start);
        break;
      case Twinkle: // random LEDs turning on and off with certain color
        m_toAnimate =
            new TwinkleAnimation(
                m_red, m_green, m_blue, m_white, m_speed, m_end, TwinklePercent.Percent6, m_start);
        break;
      case TwinkleOff: // twinkle in reverse
        m_toAnimate =
            new TwinkleOffAnimation(
                m_red,
                m_green,
                m_blue,
                m_white,
                m_speed,
                LED.LEDcount,
                TwinkleOffPercent.Percent100);
        break;
      case Solid:
        m_toAnimate = null;
        break;
      default:
        //        System.out.println("Incorrect animation type provided to changeAnimation()
        // method");
        break;
    }
  }

  /**
   * Will set LEDs a coordinated color for an action and a simpler way to define the states'
   * patterns. To add more than one pattern, another declaration of the setPattern() method is
   * needed.
   *
   * <p>NOTICE: The candle can only play one animation at a time. It will refer to the last instance
   * of setPattern() and default to the animation set in the call; the rest of the defined patterns
   * will be set to the solid animation.
   *
   * @param state the current robot state (to add or check check listed states, go to LED constants)
   */
  public void expressState(SUBSYSTEM_STATES state) {
    if (state != currentRobotState) {
      switch (state) {
        case REVED:
          setPattern(LED.blue, 0, 0, ANIMATION_TYPE.Solid, 0, 34); // Solid Blue
          break;
        case UNREVED:
          setPattern(LED.white, 125, 0.5, ANIMATION_TYPE.ColorFlow, 0, 34); // Flowing White
          break;
        case INTAKED:
          setPattern(LED.orange, 0, 0, ANIMATION_TYPE.Solid, 0, 34); // Solid Orange
          break;
        case INTAKING:
          setPattern(LED.orange, 0, 0, ANIMATION_TYPE.Strobe, 0, 34); // Strobing Orange
          break;
        case ENABLED:
          setPattern(LED.green, 0, 0, ANIMATION_TYPE.Strobe, 0, 34); // Strobing Green
          break;
        case CLIMBING:
          setPattern(LED.purple, 0, 0, ANIMATION_TYPE.Rainbow, 0, 34); // Flowing Rainbow
          break;
        case SETUP_READY:
          setPattern(LED.green, 0, 0, ANIMATION_TYPE.Solid, 0, 34); // Solid Green
          break;
        case SETUP_LOCALIZED:
          setPattern(LED.white, 0, 0, ANIMATION_TYPE.Solid, 0, 34); // Solid White
          break;
        case DISABLED:
          setPattern(LED.red, 0, 0.125, ANIMATION_TYPE.ColorFlowLong, 0, 34); // Solid Red
          break;
        case LOCKED_IN:
          setPattern(LED.turquoise, 0, 1, ANIMATION_TYPE.Larson, 0, 34); // Bouncing Turquoise
          break;
        default:
          break;
      }
      currentRobotState = state;
    }
  }

  public Color8Bit getColor() {
    return m_color;
  }

  private void updateLogger() {
    Logger.recordOutput("LEDSubsystem/LED Mode", currentRobotState.toString());

    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.DEBUG.get()) {
      Logger.recordOutput("LEDSubsystem/LED RED", m_color.red);
      Logger.recordOutput("LEDSubsystem/LED GREEN", m_color.green);
      Logger.recordOutput("LEDSubsystem/LED BLUE", m_color.blue);
      Logger.recordOutput("LEDSubsystem/LED WHITE", m_white);
      Logger.recordOutput("LEDSubsystem/LED SPEED", m_speed);
    }
  }

  @Override
  public void periodic() {
    // null indicates that the animation is "Solid"
    if (m_toAnimate == null && !setSolid) {
      setSolid = true;
      m_candle.setLEDs(m_red, m_green, m_blue, m_white, m_start, m_end);
    } else {
      setSolid = false;
      m_candle.animate(m_toAnimate); // setting the candle animation to m_animation if not null
    }
    SmartDashboard.putString("LED Mode", currentRobotState.toString());

    if (ROBOT.logMode.get() <= ROBOT.LOG_MODE.NORMAL.get()) updateLogger();
  }
}
