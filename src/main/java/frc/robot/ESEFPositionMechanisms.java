package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public interface ESEFPositionMechanisms {
    public void setElevatorHeightSetpoint(Distance height);
    public Distance getElevatorHeight();
    public void setShoulderAngleSetpoint(Angle angle);
    public Angle getShoulderAngle();
}
