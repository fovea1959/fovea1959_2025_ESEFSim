package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ESEFPosition {
  Distance elevatorHeight;
  Angle shoulderAngle;

  public ESEFPosition() {
    elevatorHeight = Meters.of(0);
    shoulderAngle = Degrees.of(0);
  }

  public ESEFPosition(Distance elevatorHeight, Angle shoulderAngle) {
    this.elevatorHeight = elevatorHeight;
    this.shoulderAngle = shoulderAngle;
  }

  public Distance getElevatorHeight() {
    return elevatorHeight;
  }

  public Angle getShoulderAngle() {
    return shoulderAngle;
  }

  public String toString() {
    return super.toString() + " " + elevatorHeight + shoulderAngle;
  }

}
