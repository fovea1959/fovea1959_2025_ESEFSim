package frc.robot.subsystems.esefsubsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ESEFPositionController {
  /* --------------------------------------------------------------------- */
  /* Don't touch anything from here to where it says                       */
  /* "You can add and modify the code below."                              */
  /* --------------------------------------------------------------------- */
  final ESEFElevatorMechanism elevatorMechanism;
  final ESEFShoulderMechanism shoulderMechanism;
  
  ESEFPosition ultimateSetpoint;
  ESEFPosition intermediateSetpoint;

  ESEFMech ultimateSetpointMech = new ESEFMech();
  ESEFMech intermediateSetpointMech = new ESEFMech();
  ESEFMech actualPositionMech = new ESEFMech();

  public ESEFPositionController(ESEFElevatorMechanism elevatorMechanism, ESEFShoulderMechanism shoulderMechanism) {
    this.elevatorMechanism = elevatorMechanism;
    this.shoulderMechanism = shoulderMechanism;

    intermediateSetpoint = new ESEFPosition();

    SmartDashboard.putData("frc3620/ESEF/setpointMech", ultimateSetpointMech.getMech());
    SmartDashboard.putData("frc3620/ESEF/intermediateSetpointMech", intermediateSetpointMech.getMech());
    SmartDashboard.putData("frc3620/ESEF/actualPositionMech", actualPositionMech.getMech());

    setPosition(new ESEFPosition(elevatorMechanism.getCurrentHeight(), shoulderMechanism.getCurrentAngle()));
    setElevatorHeightSetpoint(elevatorMechanism.getCurrentHeight());
    setShoulderAngleSetpoint(shoulderMechanism.getCurrentAngle());
  }

  public void setPosition (ESEFPosition position) {
    System.out.println ("setting position: " + position);
    ultimateSetpoint = position;
    ultimateSetpointMech.setShoulderAngle(ultimateSetpoint.shoulderAngle);
    ultimateSetpointMech.setElevatorHeight(ultimateSetpoint.elevatorHeight);
    recalculate();
  }

  public void bumpElevatorHeight(Distance delta) {
    ultimateSetpoint.elevatorHeight = ultimateSetpoint.elevatorHeight.plus(delta);
    ultimateSetpointMech.setElevatorHeight(ultimateSetpoint.elevatorHeight);
    recalculate();
  }

  public void bumpShoulderAngle(Angle delta) {
    ultimateSetpoint.shoulderAngle = ultimateSetpoint.shoulderAngle.plus(delta);
    ultimateSetpointMech.setShoulderAngle(ultimateSetpoint.shoulderAngle);
    recalculate();
  }

  void setElevatorHeightSetpoint(Distance d) {
    intermediateSetpoint.elevatorHeight = d;
    elevatorMechanism.setSetpoint(d);
    intermediateSetpointMech.setElevatorHeight(d);
  }

  void setShoulderAngleSetpoint(Angle a) {
    intermediateSetpoint.shoulderAngle = a;
    shoulderMechanism.setSetpoint(a);
    intermediateSetpointMech.setShoulderAngle(a);
  }

  public void periodic() {
    Distance currentHeight = elevatorMechanism.getCurrentHeight();
    Angle currentShoulderAngle = shoulderMechanism.getCurrentAngle();
    actualPositionMech.setElevatorHeight(currentHeight);
    actualPositionMech.setShoulderAngle(currentShoulderAngle);
    tweakIntermediateSetpoints(currentHeight, currentShoulderAngle);
  }

  /* --------------------------------------------------------------------- */
  /* You can add and modify the code below.                                */
  /* --------------------------------------------------------------------- */

  /**
   * This is called whenever we have a new setPoint.
   */
  void recalculate() {
    setElevatorHeightSetpoint(ultimateSetpoint.getElevatorHeight());
    setShoulderAngleSetpoint(ultimateSetpoint.getShoulderAngle());
  }

  /**
   * look at current mechanism positions and change individual mechanism setpoints if necessary.
   */
  void tweakIntermediateSetpoints(Distance currentHeight, Angle currentShoulderAngle) {
  }

}
