// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoulder extends SubsystemBase {
  // The P gain for the PID controller that drives this arm.
  private double m_shoulderKp = Constants.kShoulderDefaultKp;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_shoulderGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  private final PIDController m_controller = new PIDController(m_shoulderKp, 0, 0);
  private final Encoder m_encoder =
      new Encoder(Constants.kShoulderEncoderAChannel, Constants.kShoulderEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(Constants.kShoulderMotorPort);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_shoulderSim =
      new SingleJointedArmSim(
          m_shoulderGearbox,
          Constants.kShoulderReduction,
          SingleJointedArmSim.estimateMOI(Constants.kShoulderLength, Constants.kShoulderMass),
          Constants.kShoulderLength,
          Constants.kMinAngleRads,
          Constants.kMaxAngleRads,
          true,
          0,
          Constants.kShoulderEncoderDistPerPulse,
          0.0 // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final MechanismLigament2d m_mech2d;

  /** Subsystem constructor. */
  public Shoulder(MechanismLigament2d mech2d) {
    m_mech2d = mech2d;
    m_encoder.setDistancePerPulse(Constants.kShoulderEncoderDistPerPulse);
  }

  public void periodic() {
    double actualRadians = m_encoder.getDistance();
    var pidOutput = m_controller.calculate(actualRadians);
    m_motor.setVoltage(pidOutput);
    SmartDashboard.putNumber("shoulder.pidOutput", pidOutput);
    SmartDashboard.putNumber("shoulder.pidActual", actualRadians);
    SmartDashboard.putNumber("shoulder.pidSetpoint", m_controller.getSetpoint());

    double actualDegrees = Units.radiansToDegrees(actualRadians);

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_mech2d.setAngle(actualDegrees);

    double simRads = m_shoulderSim.getAngleRads();
    double simDegrees = Units.radiansToDegrees(simRads);
    SmartDashboard.putNumber("shoulder.positionDegrees", actualDegrees);
    SmartDashboard.putNumber("shoulder.simRadians", simRads);
    SmartDashboard.putNumber("shoulder.simDegrees", simDegrees);
  }

  public void setSetpoint(Angle angle) {
    m_controller.setSetpoint(angle.in(Radians));
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_shoulderSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_shoulderSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_shoulderSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_shoulderSim.getCurrentDrawAmps()));


  }

  /** Load setpoint and kP from preferences. */
  public void stop() {
    m_motor.set(0.0);
  }
}
