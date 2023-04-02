package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.drivetrain.config.DriverConfig;
import java.util.function.DoubleSupplier;

public class DriveInput implements DoubleSupplier {
  private SlewRateLimiter limiter = new SlewRateLimiter(50);
  private DoubleSupplier in;
  private InputType type;
  private static boolean highGear;
  private DriverConfig cfg;
  private int invert;
  private double prevInput;
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  public DriveInput(DoubleSupplier in, InputType type, DriverConfig cfg) {
    this(in, type, cfg, false);
  }

  public DriveInput(DoubleSupplier in, InputType type, DriverConfig cfg, boolean isX) {
    this.in = in;
    this.cfg = cfg;
    this.type = type;
    invert = isX ? -1 : 1;
  }

  public DriveInput(
      DoubleSupplier xDoubleSupplier, DoubleSupplier yDoubleSupplier, DriverConfig cfg) {
    xSupplier = xDoubleSupplier;
    ySupplier = yDoubleSupplier;
    this.cfg = cfg;
    type = InputType.ABSOLUTE_ROTATION;
  }

  public void resetPresisent() {
    prevInput = 0;
  }

  @Override
  public double getAsDouble() {

    if (type == InputType.ABSOLUTE_ROTATION) {

      if (Math.hypot(ySupplier.getAsDouble(), xSupplier.getAsDouble()) < DriverConfig.rotDeadband) {
        return prevInput;
      }
      prevInput = Math.atan2(-ySupplier.getAsDouble(), -xSupplier.getAsDouble());
      return prevInput;
    }

    double percentage = type.getMult(highGear, cfg);
    double limited = limiter.calculate(in.getAsDouble() * invert * percentage);
    limited = MathUtil.applyDeadband(limited, cfg.deadband);
    double output = Math.copySign(limited * limited, limited);
    return output;
  }

  public static void setToHighGear(boolean in) {
    highGear = in;
  }

  public static void toggleGear() {
    highGear = !highGear;
  }

  public enum InputType {
    TRANSLATION(0),
    ROTATION(1),
    ABSOLUTE_ROTATION(2);
    private int id;

    private InputType(int id) {
      this.id = id;
    }

    public double getMult(boolean highGear, DriverConfig cfg) {
      switch (id) {
          // The `default` behavior is intentionally falling through to the `case 0` behavior here.
        default:
        case 0:
          if (highGear) return -cfg.highGearMaxSped;
          else return -cfg.lowGearMaxSped;
        case 1:
          if (highGear) return cfg.highGearTurnPercent;
          else return cfg.lowgearTurnPercent;
      }
    }
  }
}
