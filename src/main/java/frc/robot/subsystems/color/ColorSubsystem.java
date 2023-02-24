package frc.robot.subsystems.color;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSubsystem extends SubsystemBase {
  private final CANdle gamePieceCandle;
  private final CANdle gridPositionCandle;

  public final PieceColoring pieceColoring = new PieceColoring();
  public final PositionColoring positionColoring = new PositionColoring();
  public final MorseCodeAnimation morseCodeAnimation = new MorseCodeAnimation();
  private final BooleanSubscriber isCubeSubscriber;
  private boolean lastColor = true;

  private final RainbowAnimation rainbowAnimation;

  public ColorSubsystem(int gamePieceID, int gridSelectID, BooleanTopic isCubeTopic) {

    gamePieceCandle = new CANdle(gamePieceID);
    gridPositionCandle = new CANdle(gridSelectID);
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

    rainbowAnimation = new RainbowAnimation(1, Constants.RAINBOW_ANIMATION_SPEED, 64);
  }

  public void startRainbowAnimation() {
    gamePieceCandle.animate(rainbowAnimation);
    gridPositionCandle.animate(rainbowAnimation);
  }

  public void stopRainbowAnimation() {
    gamePieceCandle.clearAnimation(0);
    gridPositionCandle.clearAnimation(0);
  }

  @Override
  public void periodic() {
    if (lastColor != isCubeSubscriber.get()) {
      this.pieceColoring.update();
      lastColor = isCubeSubscriber.get();
    }
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
    gridPositionCandle.setLEDs(rgb.red, rgb.green, rgb.blue);
  }

  private void writeLEDsGridPosition(RGB rgb, int startIdx, int count) {
    gridPositionCandle.setLEDs(rgb.red, rgb.green, rgb.blue, rgb.white, startIdx, count);
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
}
