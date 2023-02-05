package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class Wrist {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final ProfiledPIDController ppc;
  private final double gearRatio;

  public Wrist(int wristID) {
    motor = new CANSparkMax(wristID, MotorType.kBrushless);
    ppc =
        new ProfiledPIDController(
            Constants.wristKp,
            0,
            0,
            new TrapezoidProfile.Constraints(Constants.wristMaxVelo, Constants.wristMaxAccel));
    encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    gearRatio = Constants.wristGearRatio;
  }

  public void periodic() {
    motor.set(ppc.calculate(encoder.getPosition()) + computeArbitraryFeetForward());
  }

  public void setPositionDegrees(double degrees) {
    ppc.setGoal(degrees);
  }
  // Shouldn't this take into account te rotation of the arm as a whole?
  private double computeArbitraryFeetForward() {
    double theta = encoder.getPosition() * Constants.armRotationalTicksToRadians;
    return (Math.cos(theta) * Constants.wristMomentOfInertia * 9.8)
        / (Constants.stalledTorque * Constants.wristGearRatio);
  }
}
