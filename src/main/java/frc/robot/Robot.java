// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Elevator;

/** This is a sample program to demonstrate the use of elevator simulation. */
public class Robot extends TimedRobot {

  private final Elevator m_elevator;
  private final Shoulder m_shoulder;

  private final Mech esefMechanism;

  public Robot() {
    esefMechanism = new Mech();

    m_elevator = new Elevator(esefMechanism.elevatorMech2d);
    m_shoulder = new Shoulder(esefMechanism.shoulderMech2d);

    if (m_elevator == null) {
      SmartDashboard.putData("ee max", Commands.runOnce(() -> esefMechanism.elevatorMech2d.setLength(Constants.kElevatorMaxHeightMeters)));
      SmartDashboard.putData("ee min", Commands.runOnce(() -> esefMechanism.elevatorMech2d.setLength(Constants.kElevatorMinHeightMeters)));
    } else {
      SmartDashboard.putData("ee max", Commands.runOnce(() -> m_elevator.setSetpoint(Constants.kElevatorMaxHeightMeters)));
      SmartDashboard.putData("ee min", Commands.runOnce(() -> m_elevator.setSetpoint(Constants.kElevatorMinHeightMeters)));
    }

    if (m_shoulder == null) {
      SmartDashboard.putData("ss 0", Commands.runOnce(() -> esefMechanism.shoulderMech2d.setAngle(0)));
      SmartDashboard.putData("ss 45", Commands.runOnce(() -> esefMechanism.shoulderMech2d.setAngle(45)));
      SmartDashboard.putData("ss 135", Commands.runOnce(() -> esefMechanism.shoulderMech2d.setAngle(135)));
    } else {
      SmartDashboard.putData("ss 0", Commands.runOnce(() -> m_shoulder.setSetpoint(Angle.ofRelativeUnits(0.0, Degree))));
      SmartDashboard.putData("ss 45", Commands.runOnce(() -> m_shoulder.setSetpoint(Angle.ofRelativeUnits(45.0, Degree))));
      SmartDashboard.putData("ss 135", Commands.runOnce(() -> m_shoulder.setSetpoint(Angle.ofRelativeUnits(135.0, Degree))));
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("mech.elevator.length", esefMechanism.elevatorMech2d.getLength());
  }
}
