// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ESEFElevatorMechanism extends SubsystemBase {
  // This gearbox represents a gearbox containing 4 Vex 775pro motors.
  private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.kElevatorKp,
          Constants.kElevatorKi,
          Constants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);
  private final Encoder m_encoder =
      new Encoder(Constants.kElevatorEncoderAChannel, Constants.kElevatorEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kElevatorMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          Constants.kElevatorGearing,
          Constants.kCarriageMass,
          Constants.kElevatorDrumRadius,
          Constants.kElevatorMinHeightMeters,
          Constants.kElevatorMaxHeightMeters,
          true,
          Constants.kElevatorMinHeightMeters,
          0.01,
          0.0);
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  private final PWMSim m_motorSim = new PWMSim(m_motor);

  /** Subsystem constructor. */
  public ESEFElevatorMechanism() {
    m_encoder.setDistancePerPulse(Constants.kElevatorEncoderDistPerPulse);

    m_controller.setGoal(Constants.kElevatorMinHeightMeters);
    System.out.println ("elevation before = " + getCurrentHeight());
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    System.out.println ("elevation after  = " + getCurrentHeight());
  }

  public void periodic() {
    double p = m_encoder.getDistance();
    double pidOutput = m_controller.calculate(p);
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    double motorOutput = pidOutput + feedforwardOutput;

    if (DriverStation.isEnabled()) {
      motorOutput = MathUtil.clamp(motorOutput, -3, 3);
      m_motor.setVoltage(motorOutput);
    } else {
      m_motor.stopMotor();
    }

    SmartDashboard.putNumber("elevator.actual", p);
    SmartDashboard.putNumber("elevator.pidSetpoint", m_controller.getSetpoint().position);
    SmartDashboard.putNumber("elevator.pidOutput", pidOutput);
    SmartDashboard.putNumber("elevator.motorOutput", motorOutput);
    SmartDashboard.putString("elevator.goal", m_controller.getSetpoint().toString());
  }

  public void setSetpoint(Distance height) {
    m_controller.setGoal(height.in(Meters));
  }

  public Distance getCurrentHeight() {
    return Meters.of(m_encoder.getDistance());
  }

  /** Advance the simulation. */
  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));
  }
}
