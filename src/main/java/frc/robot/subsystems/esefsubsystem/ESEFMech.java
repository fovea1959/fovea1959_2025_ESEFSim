package frc.robot.subsystems.esefsubsystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ESEFMech {
	// Create a Mechanism2d visualization of ESEF
	private final Mechanism2d m_mech2d = new Mechanism2d(
			3 * Constants.kShoulderLength,
			(Constants.kShoulderLength + Constants.kElevatorMaxHeightMeters) * 1.1);
	private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ESEF", 1.5 * Constants.kShoulderLength, 0);
	private final MechanismLigament2d elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d(
			"Elevator",
			Constants.kElevatorMinHeightMeters,
			90,
			40,
			new Color8Bit(Color.kBlue)));
	private final MechanismLigament2d shoulderMech2d = elevatorMech2d.append(new MechanismLigament2d(
			"Shoulder",
			Constants.kShoulderLength,
			Constants.kShoulderDefaultSetpointDegrees,
			20,
			new Color8Bit(Color.kYellow)));

	public ESEFMech() {
	}

	public void setElevatorHeight(Distance d) {
		elevatorMech2d.setLength(d.in(Meters));
	}

	public void setShoulderAngle(Angle a) {
		shoulderMech2d.setAngle(a.in(Degrees) - 90);
	}

	public Mechanism2d getMech() {
		return m_mech2d;
	}

}
