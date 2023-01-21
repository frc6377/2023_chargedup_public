package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

class ActiveArmExtender implements ArmExtender {
  // Extend PID values
  private static final double EXTEND_PPC_KP = 0.4;
  private static final double EXTEND_PPC_KI = 0;
  private static final double EXTEND_PPC_KD = 0;
  private static final double EXTEND_PPC_MAX_V = 30;
  private static final double EXTEND_PPC_MAX_ACCEL = 30;
  private static final int EXTEND_COUNTS_PER_REV = 42;
  private static final int EXTEND_SMART_CURRENT_LIMIT = 40;

  private final CANSparkMax extendMotor;
  private final RelativeEncoder extendEncoder;
  private final ProfiledPIDController extendPpc;

  public ActiveArmExtender(final int extendId) {
    extendPpc =
        new ProfiledPIDController(
            EXTEND_PPC_KP,
            EXTEND_PPC_KI,
            EXTEND_PPC_KD,
            new TrapezoidProfile.Constraints(EXTEND_PPC_MAX_V, EXTEND_PPC_MAX_ACCEL));
    extendMotor = new CANSparkMax(extendId, MotorType.kBrushless);
    extendMotor.restoreFactoryDefaults();
    extendEncoder =
        extendMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, EXTEND_COUNTS_PER_REV);

    extendMotor.setSmartCurrentLimit(EXTEND_SMART_CURRENT_LIMIT);
  }

  public void periodic() {
    extendMotor.set(extendPpc.calculate(extendEncoder.getPosition()));
    SmartDashboard.putNumber("extender encoder pos", extendEncoder.getPosition());
    SmartDashboard.putNumber("extender ppc error", extendPpc.getPositionError());
    SmartDashboard.putNumber("extender setpoint", extendPpc.getSetpoint().position);
    SmartDashboard.putNumber("extender goal", extendPpc.getGoal().position);
  }

  public void setLength(double position) {
    // controller.setReference(position, CANSparkMax.ControlType.kPosition);
    extendPpc.setGoal(position);
  }

  public double getLength() {
    return extendEncoder.getPosition() * Constants.armLengthTicksToMeters
        + Constants.armLengthAtZeroTicks;
  }
}
