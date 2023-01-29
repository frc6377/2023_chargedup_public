package frc.robot.subsystems.color;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSubsystem extends SubsystemBase {
  private final int maxLights = 68;
  private final CANdle candle;

  public final PieceColoring pieceColoring = new PieceColoring();
  public final PositionColoring positionColoring = new PositionColoring();
  public final MorseCodeAnimation morseCodeAnimation = new MorseCodeAnimation();
  private int periodicCount = 0;
  private boolean runningAnimation = true;

  public ColorSubsystem(int id) {
    candle = new CANdle(id);

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
  }

  public void periodic() {
    periodicCount++;
    if (runningAnimation && periodicCount % 10 == 0 && periodicCount < 2000) {
      this.morseCodeAnimation.AdvanceAnimation();
    }
  }

  private void clearLEDs() {
    writeLED(RGB.BLACK);
  }

  private void writeLED(RGB rgb) {
    candle.setLEDs(rgb.red, rgb.green, rgb.blue);
  }

  private void writeLED(RGB rgb, int startIdx, int count) {
    candle.setLEDs(rgb.red, rgb.green, rgb.blue, rgb.white, startIdx, count);
  }

  public class PieceColoring {
    private boolean flashing = true;
    private int colorSelect = 0;
    private int lightsOn = 0;

    public void toggleHeight() {
      this.lightsOn++;
      this.lightsOn %= 3;
      update();
    }

    public void toggleColor() {
      this.colorSelect = (this.colorSelect + 1) % 3;
      update();
    }

    public void startFlashing() {
      flashing = true;
      update();
    }

    public void stopFlashing() {
      flashing = false;
      update();
    }

    private void update() {
      runningAnimation = false;
      final int LightBlock = 5;
      final int FirstSpacer = 3;
      final int SecondSpacer = 4;

      RGB color = RGB.BLACK;
      if (colorSelect > 0) {
        color = colorSelect == 1 ? RGB.YELLOW : RGB.PURPLE;
      }

      for (int i = 0; i < maxLights; i++) {
        int pos = i % LightBlock;

        // This code is a little tricky so how it how it works
        // We want lights to be in groups of 5,
        // for low, we want 1 lights and four dark
        // for mid, we want 2 lights and three dark
        // for high, we want 3 lights and two dark
        if (pos == FirstSpacer || pos == SecondSpacer || pos > lightsOn) {
          writeLED(RGB.BLACK, i, 1);
        } else {
          writeLED(color, i, 1);
        }
      }
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

    public void AdvanceAnimation() {
      if (periodicCount / 10 >= animation.length) periodicCount = 0;
      if (animation[periodicCount / 10] == 1) writeLED(RGB.HOWDY_BLUE);
      else writeLED(RGB.BLACK);
    }
  }

  public class PositionColoring {
    private final int MaxPosition = 9;

    private int cursor = 0; // 0 to 8

    public void Increment() {
      cursor = (cursor + 1) % MaxPosition;
      clearLEDs();
      update();
    }

    public void Decrement() {
      cursor = (cursor + (MaxPosition - 1)) % MaxPosition;
      clearLEDs();
      update();
    }

    public void update() {
      runningAnimation = false;
      final int StartIndex = 8;
      final int Spacer1 = StartIndex + 3;
      final int Spacer2 = StartIndex + 7;

      // 8 9 10 [11] 12 13 14 [15] 16 17 18

      clearLEDs();
      writeLED(RGB.WHITE, Spacer1, 1);
      writeLED(RGB.WHITE, Spacer2, 1);

      int cursorLocation = cursor + StartIndex + (int) (cursor / 3);

      if (cursor % 3 == 0) {
        writeLED(RGB.RED, cursorLocation, 1);
      }
      if (cursor % 3 == 1) {
        writeLED(RGB.YELLOW, cursorLocation, 1);
      }
      if (cursor % 3 == 2) {
        writeLED(RGB.GREEN, cursorLocation, 1);
      }
    }
  }
}
