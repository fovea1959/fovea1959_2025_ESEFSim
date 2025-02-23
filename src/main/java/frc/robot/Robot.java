// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.esefsubsystem.Constants;
import frc.robot.subsystems.esefsubsystem.ESEFElevatorMechanism;
import frc.robot.subsystems.esefsubsystem.ESEFPosition;
import frc.robot.subsystems.esefsubsystem.ESEFPositionController;
import frc.robot.subsystems.esefsubsystem.ESEFShoulderMechanism;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {

  private final ESEFElevatorMechanism m_elevator;
  private final ESEFShoulderMechanism m_shoulder;

  private final ESEFPositionController esefPositionController;

  public Robot() {
    DogLog.setOptions(new DogLogOptions().withCaptureNt(true));

    m_elevator = new ESEFElevatorMechanism();
    m_shoulder = new ESEFShoulderMechanism();

    esefPositionController = new ESEFPositionController(m_elevator, m_shoulder);

    ESEFPosition initialPosition = new ESEFPosition(Meters.of(0), Degrees.of(90));
    esefPositionController.setPosition(initialPosition);

    SmartDashboard.putData("pos 0+90", Commands.runOnce(() -> esefPositionController.setPosition(new ESEFPosition(Meters.of(0), Degrees.of(90)))));
    SmartDashboard.putData("pos 0.8-20", Commands.runOnce(() -> esefPositionController.setPosition(new ESEFPosition(Meters.of(0.8), Degrees.of(-20)))));
    SmartDashboard.putData("pos 1.5+0", Commands.runOnce(() -> esefPositionController.setPosition(new ESEFPosition(Meters.of(1.5), Degrees.of(0)))));
    SmartDashboard.putData("pos 2.0+135", Commands.runOnce(() -> esefPositionController.setPosition(new ESEFPosition(Meters.of(2.0), Degrees.of(135)))));

    SmartDashboard.putData("e +0.1", Commands.runOnce(() -> esefPositionController.bumpElevatorHeight(Meters.of(0.1))));
    SmartDashboard.putData("e -0.1", Commands.runOnce(() -> esefPositionController.bumpElevatorHeight(Meters.of(-0.1))));
    SmartDashboard.putData("s +5.0", Commands.runOnce(() -> esefPositionController.bumpShoulderAngle(Degrees.of(5))));
    SmartDashboard.putData("s -5.0", Commands.runOnce(() -> esefPositionController.bumpShoulderAngle(Degrees.of(-5))));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    esefPositionController.periodic();
  }

  // get rid of "Default xxxxxxx method... Override me!" messages
  @Override public void teleopPeriodic() { }
  @Override public void disabledPeriodic() { }
  @Override public void simulationPeriodic() { }
}
