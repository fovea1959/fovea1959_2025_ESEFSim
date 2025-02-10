// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class Mech {
    // Create a Mechanism2d visualization of the elevator
    private final Mechanism2d m_mech2d = new Mechanism2d(
        3 * Constants.kShoulderLength,
        (Constants.kShoulderLength + Constants.kElevatorMaxHeightMeters) * 1.1
    );
    private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ESEF", 1.5 * Constants.kShoulderLength, 0);
    public final MechanismLigament2d elevatorMech2d = m_mech2dRoot.append(new MechanismLigament2d(
        "Elevator",
        Constants.kElevatorMinHeightMeters,
        90, 
        40, 
        new Color8Bit(Color.kBlue)
    ));
    public final MechanismLigament2d shoulderMech2d = elevatorMech2d.append(new MechanismLigament2d(
        "Shoulder",
        Constants.kShoulderLength,
        Constants.kShoulderDefaultSetpointDegrees,
        20,
        new Color8Bit(Color.kYellow)
    ));

    public Mech() {
        // Publish Mechanism2d to SmartDashboard
        // To view the Elevator visualization, select Network Tables -> SmartDashboard
        // -> Elevator Sim
        SmartDashboard.putData("Elevator Sim", m_mech2d);

    }

}
