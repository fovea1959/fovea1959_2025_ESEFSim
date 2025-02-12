// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.esefsubsystem;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kElevatorMotorPort = 0;
  public static final int kElevatorEncoderAChannel = 0;
  public static final int kElevatorEncoderBChannel = 1;

  public static final double kElevatorKp = 5;
  public static final double kElevatorKi = 0;
  public static final double kElevatorKd = 0;

  public static final double kElevatorkS = 0.0; // volts (V)
  public static final double kElevatorkG = 0.762; // volts (V)
  public static final double kElevatorkV = 0.762; // volt per velocity (V/(m/s))
  public static final double kElevatorkA = 0.0; // volt per acceleration (V/(m/s²))

  public static final double kElevatorGearing = 10.0;
  public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
  public static final double kCarriageMass = 1.0; // kg

  public static final double kElevatorMinHeightMeters = 0.0;
  public static final double kElevatorMaxHeightMeters = 2.0;

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  public static final double kElevatorEncoderDistPerPulse =
      2.0 * Math.PI * kElevatorDrumRadius / 4096;

  public static final int kShoulderMotorPort = 8;
  public static final int kShoulderEncoderAChannel = 2;
  public static final int kShoulderEncoderBChannel = 3;

  // The P gain for the PID controller that drives this arm.
  public static final double kShoulderDefaultKp = 50.0;
  public static final double kShoulderDefaultSetpointDegrees = 75.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kShoulderEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kShoulderReduction = 200;
  public static final double kShoulderMass = 8.0; // Kilograms
  public static final double kShoulderLength = Units.inchesToMeters(18);
  public static final double kMinAngleRads = Units.degreesToRadians(-45);
  public static final double kMaxAngleRads = Units.degreesToRadians(170);
}
