package ArmSubsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.arm.ArmSubsystem;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

public class ArmSubsystemTest {
  // 22.5 is 90 degrees
  @ParameterizedTest
  @CsvSource(
      value = {"22.5:0:0", "22.5:5000:0"}, // , "9.3:0:0", "25:50:0"},
      delimiter = ':')
  void TestA(double rotation, double extend, double expected) {
    var actual = ArmSubsystem.rotationArbitraryFeetForward(rotation, extend);
    assertEquals(expected, actual, 0.0001);
  }
}
