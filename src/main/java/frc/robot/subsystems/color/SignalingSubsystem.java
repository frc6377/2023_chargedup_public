package frc.robot.subsystems.color;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.networktables.DeltaBoard;
import frc.robot.subsystems.color.patterns.BIFlag;
import frc.robot.subsystems.color.patterns.FireFlyPattern;
import frc.robot.subsystems.color.patterns.PatternNode;
import frc.robot.subsystems.color.patterns.TransFlag;
import java.util.function.Consumer;

public class SignalingSubsystem extends SubsystemBase {
  private static final int patternUpdateFrequency = 10;

  private final CANdle gamePieceCandle;
  // private final CANdle gridPositionCandle;

  private static final int numberOfLEDS = 70;

  private int tick;
  private int patternTick = 0;
  private DisablePattern disablePattern = DisablePattern.getRandom();

  private GamePieceMode mode;
  private boolean hasGamePiece;
  private Timer flashTimer = new Timer();
  private boolean flashOn = true;
  private final Consumer<Double> driverRumbleConsumer;

  private final RainbowAnimation rainbowAnimation;

  public SignalingSubsystem(
      int gamePieceID, final GamePieceMode gamePieceMode, Consumer<Double> driverRumbleConsumer) {

    this.mode = gamePieceMode;
    this.driverRumbleConsumer = driverRumbleConsumer;

    gamePieceCandle = new CANdle(gamePieceID);
    // gridPositionCandle = new CANdle(gridSelectID);
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

    TransFlag.numberOfLEDS = numberOfLEDS;
    FireFlyPattern.numberOfLEDS = numberOfLEDS;
    BIFlag.numberOfLEDS = numberOfLEDS;
    rainbowAnimation = new RainbowAnimation(1, Constants.RAINBOW_ANIMATION_SPEED, 64);
  }

  public void setGamePiece(GamePieceMode mode) {
    this.mode = mode;
    updateLEDs();
  }

  public void updateLEDs() {
    if (hasGamePiece) writeLEDsGamePiece(RGB.HOWDY_BLUE);
    else {
      writeLEDsGamePiece(mode.color());
    }
    flashTimer.start();
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
    if (DriverStation.isDisabled()) updatePattern();
    if (mode.shouldFlash()) {
      if (flashTimer.get() > Constants.FLASHING_TIME) {
        flashTimer.reset();
        flashOn = !flashOn;
        if (flashOn) writeLEDsGamePiece(mode.color());
        else writeLEDsGamePiece(RGB.BLACK);
      }
    }
  }

  public void hasGamePieceSignalStart() {
    hasGamePiece = true;
    driverRumbleConsumer.accept(Constants.RUMBLE_INTENSITY);
    updateLEDs();
  }

  public void hasGamePieceSignalStop() {
    hasGamePiece = false;
    driverRumbleConsumer.accept(0.0);
    updateLEDs();
  }

  private void clearLEDsGamePiece() {
    writeLEDsGamePiece(RGB.BLACK);
  }

  private void writeLEDsGamePiece(RGB rgb) {
    gamePieceCandle.setLEDs(rgb.red, rgb.green, rgb.blue);
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

    //DeltaBoard.putString("Disable Pattern", disablePattern.name());

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
