// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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

  private final ESEFPositionMechanisms esefPositionMechanisms;
  private final ESEFPositionController esefPositionController;

  private final Mech esefMechanism;

  public Robot() {
    DogLog.setOptions(new DogLogOptions().withCaptureNt(true));
    esefMechanism = new Mech();

    m_elevator = new Elevator(esefMechanism.elevatorMech2d);
    m_shoulder = new Shoulder(esefMechanism.shoulderMech2d);

    esefPositionMechanisms = new ESEFPositionMechanisms() {

      @Override
      public void setElevatorHeightSetpoint(Distance height) {
        m_elevator.setSetpoint(height.in(Meters));
      }

      @Override
      public Distance getElevatorHeight() {
        return m_elevator.getCurrentHeight();
      }

      @Override
      public void setShoulderAngleSetpoint(Angle angle) {
        m_shoulder.setSetpoint(angle);
      }

      @Override
      public Angle getShoulderAngle() {
        return m_shoulder.getCurrentAngle();
      }
      
    };

    esefPositionController = new ESEFPositionController(esefPositionMechanisms);

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
}
