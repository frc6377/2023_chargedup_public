package frc.robot.subsystems.arm;

import frc.robot.Constants;

class DisabledArmExtender implements ArmExtender {
  public DisabledArmExtender(final int extendId) {}

  public void periodic() {}

  public void setLength(double position) {}

  public double getLength() {
    return Constants.armLengthAtZeroTicks;
  }
}
