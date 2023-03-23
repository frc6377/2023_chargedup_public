package frc.robot.subsystems.color;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.networktables.DeltaBoard;
import frc.robot.subsystems.color.patterns.BIFlag;
import frc.robot.subsystems.color.patterns.FireFlyPattern;
import frc.robot.subsystems.color.patterns.PatternNode;
import frc.robot.subsystems.color.patterns.TransFlag;

public class ColorSubsystem extends SubsystemBase {
  private static final int patternUpdateFrequency = 10;

  private final CANdle gamePieceCandle;
  // private final CANdle gridPositionCandle;

  private static final int numberOfLEDS = 70;

  private int tick;
  private int patternTick = 0;
  private DisablePattern disablePattern = DisablePattern.getRandom();

  public final PieceColoring pieceColoring = new PieceColoring();
  public final PositionColoring positionColoring = new PositionColoring();
  public final MorseCodeAnimation morseCodeAnimation = new MorseCodeAnimation();
  private final BooleanSubscriber isCubeSubscriber;
  private boolean lastColor = true;

  private final RainbowAnimation rainbowAnimation;

  public ColorSubsystem(int gamePieceID, int gridSelectID, BooleanTopic isCubeTopic) {

    gamePieceCandle = new CANdle(gamePieceID);
    // gridPositionCandle = new CANdle(gridSelectID);
    this.isCubeSubscriber = isCubeTopic.subscribe(true);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    var errorcode = gamePieceCandle.configAllSettings(configAll, 100);
    if (errorcode != ErrorCode.OK) {
      System.out.println("Error initializing CANdle");
    }

    clearLEDsGamePiece();
    clearLEDsGridPosition();

    TransFlag.numberOfLEDS = numberOfLEDS;
    FireFlyPattern.numberOfLEDS = numberOfLEDS;
    BIFlag.numberOfLEDS = numberOfLEDS;
    rainbowAnimation = new RainbowAnimation(1, Constants.RAINBOW_ANIMATION_SPEED, 64);
  }

  public void startRainbowAnimation() {
    if (disablePattern != DisablePattern.RAINBOW) return;
    gamePieceCandle.animate(rainbowAnimation);
    // gridPositionCandle.animate(rainbowAnimation);
  }

  public void stopRainbowAnimation() {
    gamePieceCandle.clearAnimation(0);
    // gridPositionCandle.clearAnimation(0);
  }

  @Override
  public void periodic() {
    if (lastColor != isCubeSubscriber.get()) {
      this.pieceColoring.update();
      lastColor = isCubeSubscriber.get();
    }
    if (DriverStation.isDisabled()) updatePattern();
  }

  public void forceUpdate() {
    this.pieceColoring.update();
    lastColor = isCubeSubscriber.get();
  }

  private void clearLEDsGamePiece() {
    writeLEDsGamePiece(RGB.BLACK);
  }

  private void writeLEDsGamePiece(RGB rgb) {
    gamePieceCandle.setLEDs(rgb.red, rgb.green, rgb.blue);
  }

  private void writeLEDsGamePiece(RGB rgb, int startIdx, int count) {
    gamePieceCandle.setLEDs(rgb.red, rgb.green, rgb.blue, rgb.white, startIdx, count);
  }

  private void clearLEDsGridPosition() {
    writeLEDsGamePiece(RGB.BLACK);
  }

  private void writeLEDsGridPosition(RGB rgb) {
    // gridPositionCandle.setLEDs(rgb.red, rgb.green, rgb.blue);
  }

  private void writeLEDsGridPosition(RGB rgb, int startIdx, int count) {
    // gridPositionCandle.setLEDs(rgb.red, rgb.green, rgb.blue, rgb.white, startIdx, count);
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

    DeltaBoard.putString("Disable Pattern", disablePattern.name());

    switch (disablePattern) {
        // case BI_FLAG:
        //   pattern = BIFlag.getColors(patternTick);
        //   break;
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

      gamePieceCandle.setLEDs(c.red, c.green, c.blue, 0, LEDIndex, node.repeat);
      LEDIndex += node.repeat;
      patternIndex += 1;
    }
  }

  public void randomizePattern() {
    disablePattern = DisablePattern.getRandom();
  }

  public class PieceColoring {
    private void update() {

      RGB color = isCubeSubscriber.get() ? RGB.PURPLE : RGB.YELLOW;

      writeLEDsGamePiece(color);
    }
  }

  public class MorseCodeAnimation {
    // "HOWDY" in morse code
    private final int[] animation = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0,
      0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1,
      1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0
    };
  }

  public class PositionColoring {
    private final int MaxPosition = 9;

    private int cursor = 0; // 0 to 8

    public void SetPosition(int position) {
      cursor = position % MaxPosition;
      clearLEDsGridPosition();
      update();
    }

    public void Increment() {
      cursor = (cursor + 1) % MaxPosition;
      clearLEDsGridPosition();
      update();
    }

    public void Decrement() {
      cursor = (cursor + (MaxPosition - 1)) % MaxPosition;
      clearLEDsGridPosition();
      update();
    }

    public void update() {
      final int StartIndex = 8;
      final int Spacer1 = StartIndex + 3;
      final int Spacer2 = StartIndex + 7;

      // 8 9 10 [11] 12 13 14 [15] 16 17 18

      clearLEDsGridPosition();
      writeLEDsGridPosition(RGB.WHITE, Spacer1, 1);
      writeLEDsGridPosition(RGB.WHITE, Spacer2, 1);

      int cursorLocation = cursor + StartIndex + (int) (cursor / 3);

      if (cursor % 3 == 0) {
        writeLEDsGridPosition(RGB.RED, cursorLocation, 1);
      }
      if (cursor % 3 == 1) {
        writeLEDsGridPosition(RGB.YELLOW, cursorLocation, 1);
      }
      if (cursor % 3 == 2) {
        writeLEDsGridPosition(RGB.GREEN, cursorLocation, 1);
      }
    }
  }

  private enum DisablePattern {
    TRANS_FLAG,
    RAINBOW,
    FIRE_FLY;

    public static DisablePattern getRandom() {
      DisablePattern[] allPatterns = DisablePattern.values();
      return allPatterns[(int) Math.floor(Math.random() * (allPatterns.length))];
    }
  }
}
