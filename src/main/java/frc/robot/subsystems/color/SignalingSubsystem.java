package frc.robot.subsystems.color;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.color.patterns.BIFlag;
import frc.robot.subsystems.color.patterns.FireFlyPattern;
import frc.robot.subsystems.color.patterns.PatternNode;
import frc.robot.subsystems.color.patterns.TransFlag;
import java.util.ArrayList;
import java.util.function.Consumer;

public class SignalingSubsystem extends SubsystemBase {
  private static final int patternUpdateFrequency = 10;

  private final CANdle candle;

  private static final int numberOfLEDS = 70;

  private int tick;
  private int patternTick = 0;
  private DisablePattern disablePattern = DisablePattern.getRandom();

  private final IntegerSubscriber gamePieceModeSubscriber =
      NetworkTableInstance.getDefault().getIntegerTopic("GAME_PIECE_MODE").subscribe(10);
  private GamePieceMode gamePieceMode = GamePieceMode.CUBE;
  private boolean hasGamePiece;
  private final Timer flashTimer = new Timer();
  private final Timer gamePieceSignalingTimer = new Timer();
  private boolean flashOn = true;
  private final Consumer<Double> driverRumbleConsumer;

  private final RainbowAnimation rainbowAnimation;

  /**
   * Handles the lights and controller rumble
   * @param CANdleID The CANdle CAN id
   * @param driverRumbleConsumer A doubleConsumer that takes in the rumble intensity
   */
  public SignalingSubsystem(int CANdleID, Consumer<Double> driverRumbleConsumer) {
    this.driverRumbleConsumer = driverRumbleConsumer;

    candle = new CANdle(CANdleID);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;

    var errorcode = candle.configAllSettings(configAll, 100);
    if (errorcode != ErrorCode.OK) {
      System.out.println("Error initializing CANdle");
    }

    clearLEDs();

    TransFlag.numberOfLEDS = numberOfLEDS;
    FireFlyPattern.numberOfLEDS = numberOfLEDS;
    BIFlag.numberOfLEDS = numberOfLEDS;
    rainbowAnimation = new RainbowAnimation(1, Constants.RAINBOW_ANIMATION_SPEED, 64);
  }

  /**
   * Has the LEDs update their color/flashing pattern
   */
  public void updateLEDs() {
    if (hasGamePiece) writeLEDs(RGB.HOWDY_BLUE);
    else {
      writeLEDs(getColorFromGamePieceMode(gamePieceMode));
    }
    flashTimer.start();
  }

  public void startRainbowAnimation() {
    if (disablePattern != DisablePattern.RAINBOW) return;
    candle.animate(rainbowAnimation);
  }

  public void stopRainbowAnimation() {
    candle.clearAnimation(0);
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) updatePattern();

    //Checks the game piece mode and updates LEDs if it has changed
    else if (GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get()) != gamePieceMode) {
      gamePieceMode = GamePieceMode.getFromInt((int) gamePieceModeSubscriber.get());
      updateLEDs();
    } 
    else if (shouldFlash(gamePieceMode)) {
      if (flashTimer.hasElapsed(Constants.FLASHING_TIME)) {
        flashTimer.reset();
        flashOn = !flashOn;
        if (flashOn) writeLEDs(getColorFromGamePieceMode(gamePieceMode));
        else writeLEDs(RGB.BLACK);
      }
    }
    //Stops the game piece signaling when the timer ends
    if (!hasGamePiece && gamePieceSignalingTimer.hasElapsed(Constants.GAME_PIECE_SIGNALING_TIME)) {
      driverRumbleConsumer.accept(0.0);
      updateLEDs();
      gamePieceSignalingTimer.reset();
      gamePieceSignalingTimer.stop();
    }
  }

  private static RGB getColorFromGamePieceMode(GamePieceMode mode) {
    if (mode == GamePieceMode.CUBE) return RGB.PURPLE;
    if (mode == GamePieceMode.CONE || mode == GamePieceMode.SINGLE_SUBSTATION) return RGB.YELLOW;
    return RGB.WHITE;
  }

  private static boolean shouldFlash(GamePieceMode mode) {
    return mode == GamePieceMode.SINGLE_SUBSTATION;
  }

  public void hasGamePieceSignalStart() {
    hasGamePiece = true;
    driverRumbleConsumer.accept(Constants.RUMBLE_INTENSITY);
    updateLEDs();
  }

  public void hasGamePieceSignalStop() {
    hasGamePiece = false;
    gamePieceSignalingTimer.reset();
    gamePieceSignalingTimer.start();
  }

  private void clearLEDs() {
    writeLEDs(RGB.BLACK);
  }

  private void writeLEDs(RGB rgb) {
    candle.setLEDs(rgb.red, rgb.green, rgb.blue);
  }

  private void updatePattern() {
    PatternNode[] pattern;
    int patternLength;

    tick++;
    if (tick > patternUpdateFrequency) {
      tick = 0;
      patternTick++;
    } else {
      return;
    }

    switch (disablePattern) {
      case FIRE_FLY:
        pattern = FireFlyPattern.getPattern();
        patternLength = FireFlyPattern.getPatternLength();
        break;
      case RAINBOW:
        startRainbowAnimation();
        return;
      case TRANS_FLAG:
        pattern = TransFlag.getPattern();
        patternLength = TransFlag.getPatternLength();
        break;
      default:
        startRainbowAnimation();
        return;
    }
    stopRainbowAnimation();
    int patternIndex = 0;
    patternTick %= patternLength;
    int LEDIndex = -patternTick;
    while (LEDIndex < numberOfLEDS) {
      patternIndex %= pattern.length;

      PatternNode node = pattern[patternIndex];
      RGB c = pattern[patternIndex].color;

      candle.setLEDs(c.red, c.green, c.blue, 0, LEDIndex, node.repeat);
      LEDIndex += node.repeat;
      patternIndex += 1;
    }
  }

  public void randomizePattern() {
    disablePattern = DisablePattern.getRandom();
  }

  private enum DisablePattern {
    TRANS_FLAG,
    RAINBOW,
    FIRE_FLY;

    public static DisablePattern getRandom() {
      DisablePattern[] DNU = {DisablePattern.TRANS_FLAG};
      DisablePattern[] allPatterns = DisablePattern.values();
      ArrayList<DisablePattern> useable = new ArrayList<>();
      for (DisablePattern p : allPatterns) {
        boolean skip = false;
        for (DisablePattern d : DNU) {
          if (p == d) skip = true;
        }
        if (skip) continue;
        useable.add(p);
      }

      return useable.get((int) Math.floor(Math.random() * (useable.size())));
    }
  }

  public void displayCriticalError() {
    candle.setLEDs(255, 0, 0);
  }
}
