// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;

public class Score extends SequentialCommandGroup {
	private final Elevator Elevator;
	private final Fish_Hook Fish_Hook;
	private Ultrasonic ultrasonic = new Ultrasonic(0, 1);

	public Score(Elevator Elevator, Fish_Hook Fish_Hook) {
		this.Elevator = Elevator;
		this.Fish_Hook = Fish_Hook;
		addRequirements(Elevator, Fish_Hook);

		if (ultrasonic.getRangeInches() > 5) {
			addCommands(Fish_Hook.runOnce(() -> Fish_Hook.coral(0)));
		} else {
			addCommands(Fish_Hook.runOnce(() -> Fish_Hook.coral(0.2)));
		}
	}
}
