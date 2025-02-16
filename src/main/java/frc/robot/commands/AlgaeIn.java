// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class AlgaeIn extends Command {

	private final Fish_Hook m_Fishhook;

	public AlgaeIn(Fish_Hook fishhook) {
		m_Fishhook = fishhook;
		addRequirements(m_Fishhook);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		m_Fishhook.algae(-0.5);
	}

	@Override
	public void end(boolean interrupted) {
		m_Fishhook.algae(0);
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
