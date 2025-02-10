package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ESEFPositionController {
  /* --------------------------------------------------------------------- */
  /* Don't touch anything from here to where it says                       */
  /* "You can add and modify the code below."                              */
  /* --------------------------------------------------------------------- */
  ESEFPositionMechanisms mechanisms;
  ESEFPosition setpoint;

  public ESEFPositionController(ESEFPositionMechanisms esefPositionMechanisms) {
    this.mechanisms = esefPositionMechanisms;

    setpoint = new ESEFPosition(esefPositionMechanisms.getElevatorHeight(), esefPositionMechanisms.getShoulderAngle());
    recalculate();
  }

  public void setPosition (ESEFPosition position) {
    setpoint = position;
    recalculate();
  }

  public void bumpElevatorHeight(Distance delta) {
    setpoint.elevatorHeight = setpoint.elevatorHeight.plus(delta);
    recalculate();
  }

  public void bumpShoulderAngle(Angle delta) {
    setpoint.shoulderAngle = setpoint.shoulderAngle.plus(delta);
    recalculate();
  }

  /* --------------------------------------------------------------------- */
  /* You can add and modify the code below.                                */
  /* --------------------------------------------------------------------- */

  /**
   * This is called whenever we have a new setPoint.
   */
  void recalculate() {
    mechanisms.setElevatorHeightSetpoint(setpoint.getElevatorHeight());
    mechanisms.setShoulderAngleSetpoint(setpoint.getShoulderAngle());
  }

  /**
   * Look at mechanism positions and change individual mechanism setpoints if necessary.
   */
  public void periodic() {

  }

}
