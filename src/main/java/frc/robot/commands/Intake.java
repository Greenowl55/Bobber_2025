// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fish_Hook;

public class Intake extends SequentialCommandGroup {

	// private final Elevator Elevator;
	private final Fish_Hook Fish_Hook;
	private final DigitalInput photoelectricSensor = new DigitalInput(0);

	// ;)

	public Intake(Elevator m_Elevator, Fish_Hook m_Fish_Hook) {

		// Elevator = m_Elevator;
		Fish_Hook = m_Fish_Hook;
		addRequirements(m_Elevator, m_Fish_Hook);

		// addCommands(
		// 		Elevator.runOnce(() -> Elevator.setPosition(0)),
		// 		Fish_Hook.runOnce(() -> Fish_Hook.setposition(0))
		// );

		if (photoelectricSensor.get()) {
			addCommands(Fish_Hook.run(() -> Fish_Hook.coral(0.2)));
		} else {
			addCommands(Fish_Hook.run(() -> Fish_Hook.coral(0)));
		}

		// puts elevator in same position as L1 but runs coral motors
	}
}
