// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ElevatorUp extends Command {

	private final Elevator m_Elevator;

	public ElevatorUp(Elevator elevator) {
		m_Elevator = elevator;
		addRequirements(m_Elevator);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		m_Elevator.elevator(0.25);
	}

	@Override
	public void end(boolean interrupted) {
		m_Elevator.elevator(0);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public boolean runsWhenDisabled() {
		return false;
	}
}
