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
  /*
  private final Elevator m_elevator;
  private final Shoulder m_shoulder;
  */

  private final Mech esefMechanism;

  public Robot() {
    esefMechanism = new Mech();
    /*
    m_elevator = new Elevator(esefMechanism.elevatorMech2d);
    m_shoulder = new Shoulder(esefMechanism.shoulderMech2d);

    SmartDashboard.putData("e up", m_elevator.setSetpointCommand(1.0));
    SmartDashboard.putData("e down", m_elevator.setSetpointCommand(1.0));

    SmartDashboard.putData("s up", m_shoulder.setSetpointCommand(Angle.ofRelativeUnits(0.0, Degree)));
    SmartDashboard.putData("s down", m_shoulder.setSetpointCommand(Angle.ofRelativeUnits(-90.0, Degree)));
    */

    SmartDashboard.putData("ee max", Commands.runOnce(() -> esefMechanism.elevatorMech2d.setLength(Constants.kElevatorMaxHeightMeters)));
    SmartDashboard.putData("ee min", Commands.runOnce(() -> esefMechanism.elevatorMech2d.setLength(Constants.kElevatorMinHeightMeters)));

    SmartDashboard.putData("ss 0", Commands.runOnce(() -> esefMechanism.shoulderMech2d.setAngle(0)));
    SmartDashboard.putData("ss 45", Commands.runOnce(() -> esefMechanism.shoulderMech2d.setAngle(45)));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("mech.elevator.length", esefMechanism.elevatorMech2d.getLength());
  }

  @Override
  public void disabledInit() {
    /*
    // This just makes sure that our simulation code knows that the motor's off.
    m_shoulder.stop();
    m_elevator.stop();
    */
  }
}
