// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  public static final boolean isCompetition = false;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    super();
    addPeriodic(() -> m_robotContainer.setElevator(), 0.005);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  @Override
  public void robotPeriodic() {
    try {
      CommandScheduler.getInstance().run();
    } catch (OverheatedException e) {
      m_robotContainer.onFatalError();
      throw e;
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.updateLEDs();
    m_robotContainer.onDisabled();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_robotContainer.onExitDisabled();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.updateLEDs();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.updateLEDs();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.unbindShoulder();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
