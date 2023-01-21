package frc.robot.subsystems.arm;

interface ArmExtender {
  public void periodic();

  public void setLength(double position);

  public double getLength();
}
