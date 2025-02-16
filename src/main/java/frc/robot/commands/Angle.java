// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class Angle extends Command {

	private final Fish_Hook m_Angle;
	private DoubleSupplier m_speed;

	public Angle(Fish_Hook m_fish_hook, DoubleSupplier speed) {
		m_Angle = m_fish_hook;
		m_speed = speed;
		addRequirements(m_Angle);
	}


    @Override
    public void initialize() {}

	@Override
	public void execute() {
		m_Angle.tilt(m_speed.getAsDouble());
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