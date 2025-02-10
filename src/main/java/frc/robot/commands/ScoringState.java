// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

public class ScoringState {
	public enum State {
		BOTTOM(0.0, 0.0),
		L1(1.0, 35.0),
		L2(1.2, 35.0),
		L3(1.4, 35.0),
		L4(1.6, 50.0),

		BRARGE(1.8, 0.0);

		private double heightM;
		private double angleDeg;

		State(double heightM, double angleDeg) {
			this.heightM = heightM;
			this.angleDeg = angleDeg;
		}

		public double getHeightM() {
			return heightM;
		}

		public double getAngleDeg() {
			return angleDeg;
		}
	}
}
