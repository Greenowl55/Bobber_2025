// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

public class FishHookState {
	public enum State {
		IDLE(0.0),
		L1(35.0),
		L2(35.0),
		L3(35.0),
		L4(50.0);

		private double angleDeg;

		State(double angleDeg) {
			this.angleDeg = angleDeg;
		}

		public double getAngleDeg() {
			return angleDeg;
		}
	}
}
