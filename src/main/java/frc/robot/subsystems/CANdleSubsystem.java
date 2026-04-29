package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  private static final int CANDLE_ID = 32;

  // CANdle is on the roboRIO CAN bus
  private final CANdle candle = new CANdle(CANDLE_ID);

  // CANdle onboard LEDs are 0-7.
  // External LED strip starts at index 8.
  private static final int LED_START_INDEX = 0;

  // Change this to the number of LEDs you want controlled.
  // If LED_START_INDEX = 0 and LED_COUNT = 17:
  // 8 built-in LEDs + first 9 strip LEDs are controlled.
  private static final int LED_COUNT = 17;

  private static final int LED_END_INDEX = LED_START_INDEX + LED_COUNT - 1;

  private final LimelightSubsystem limelight;

  private enum LedMode {
    IDLE_LARSON,
    SEEKING_BLINK_RED,
    ON_TARGET_SOLID_YELLOW,
    AIMING_BLINK_YELLOW,
    READY_BLINK_GREEN
  }

  private LedMode currentMode = null;

  private final RGBWColor red = new RGBWColor(255, 0, 0);
  private final RGBWColor yellow = new RGBWColor(255, 180, 0);
  private final RGBWColor green = new RGBWColor(0, 255, 0);
  private final RGBWColor white = new RGBWColor(255, 255, 255);

  public CANdleSubsystem(LimelightSubsystem limelight) {
    this.limelight = limelight;

    CANdleConfiguration config = new CANdleConfiguration();

    config.LED.StripType = StripTypeValue.RGB;
    config.LED.BrightnessScalar = 0.5;

    candle.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    LedMode wantedMode = getWantedMode();

    if (wantedMode != currentMode) {
      applyMode(wantedMode);
      currentMode = wantedMode;
    }

    SmartDashboard.putString("CANdle/Mode", currentMode.toString());
  }

  private LedMode getWantedMode() {
    if (limelight.isReadyToScore()) {
      return LedMode.READY_BLINK_GREEN;
    }

    if (limelight.isOnTarget()) {
      return LedMode.ON_TARGET_SOLID_YELLOW;
    }

    if (limelight.isSeeking() && !limelight.hasTarget()) {
      return LedMode.SEEKING_BLINK_RED;
    }

    if ((limelight.isSeeking() || limelight.isAiming()) && limelight.hasTarget()) {
      return LedMode.AIMING_BLINK_YELLOW;
    }

    if (limelight.isAiming() || limelight.isDriving()) {
      return LedMode.AIMING_BLINK_YELLOW;
    }

    return LedMode.IDLE_LARSON;
  }

  private void applyMode(LedMode mode) {
    candle.clearAllAnimations();

    switch (mode) {
      case IDLE_LARSON:
        candle.setControl(
            new LarsonAnimation(LED_START_INDEX, LED_END_INDEX)
                .withColor(white)
                .withSize(3)
                .withBounceMode(LarsonBounceValue.Front)
                .withFrameRate(20));
        break;

      case SEEKING_BLINK_RED:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withColor(red)
                .withFrameRate(4));
        break;

      case ON_TARGET_SOLID_YELLOW:
        candle.setControl(
            new SolidColor(LED_START_INDEX, LED_END_INDEX)
                .withColor(yellow));
        break;

      case AIMING_BLINK_YELLOW:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withColor(yellow)
                .withFrameRate(4));
        break;

      case READY_BLINK_GREEN:
        candle.setControl(
            new StrobeAnimation(LED_START_INDEX, LED_END_INDEX)
                .withColor(green)
                .withFrameRate(6));
        break;
    }
  }
}