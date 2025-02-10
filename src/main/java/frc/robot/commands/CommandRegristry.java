// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fish_Hook;

public class CommandRegristry {

	public static void setupNamedCommands(Elevator elevator, Fish_Hook fishHook) {

		// Elevator Scoring
        NamedCommands.registerCommand("Bottom", new CMD_ScoringState(ScoringState.State.BOTTOM, elevator, fishHook));
		NamedCommands.registerCommand("L1", new CMD_ScoringState(ScoringState.State.L1, elevator, fishHook));
		NamedCommands.registerCommand("L2", new CMD_ScoringState(ScoringState.State.L2, elevator, fishHook));
		NamedCommands.registerCommand("L3", new CMD_ScoringState(ScoringState.State.L3, elevator, fishHook));
		NamedCommands.registerCommand("L4", new CMD_ScoringState(ScoringState.State.L4, elevator, fishHook));

		// Elevator Util

		// Intake
		NamedCommands.registerCommand("Intake", new Intake(elevator, fishHook));
	}
}
