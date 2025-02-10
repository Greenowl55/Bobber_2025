// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fish_Hook;

public class CMD_ScoringState extends SequentialCommandGroup {

	public CMD_ScoringState(
			ScoringState.State newScoringState, Elevator elevator, Fish_Hook fishHook) {

		addRequirements(elevator, fishHook);

		addCommands(
				// Set elevator to enum height
				elevator.runOnce(() -> elevator.setPosition(newScoringState.getHeightM())),
				fishHook.runOnce(() -> fishHook.setposition(newScoringState.getAngleDeg())));
	}
}
