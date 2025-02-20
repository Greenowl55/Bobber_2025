// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;

public class CMD_ElevatorState extends SequentialCommandGroup {

	public CMD_ElevatorState(ElevatorHight.State newScoringState, Elevator elevator) {

		addRequirements(elevator);

		addCommands(
				// Set elevator to enum height

				elevator.runOnce(() -> elevator.setPosition(newScoringState.getHight())));
	}
}
