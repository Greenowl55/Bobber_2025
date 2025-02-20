// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

public class FishHookState {
	public enum State {
		IDLE(0.0),
		Intake(0.2),
		L4(0.5),
		Algae(0.75);

		private double angle;

		State(double MotorRotations) {
			this.angle = MotorRotations;
		}

		public double getAngle() {
			return angle;
		}
	}
}
