// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FishHookState.State;
import frc.robot.subsystems.Fish_Hook;

public class CMD_FishHookState extends SequentialCommandGroup {

	public CMD_FishHookState(State newScoringState, Fish_Hook FishHook) {

		addRequirements(FishHook);

		addCommands(FishHook.runOnce(() -> FishHook.setposition(newScoringState.getAngle())));
	}
}
