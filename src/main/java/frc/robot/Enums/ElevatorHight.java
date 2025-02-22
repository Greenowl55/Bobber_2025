// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Enums;

public class ElevatorHight {
	public enum State { // 32, 48, 72
		BOTTOM(0.0),
		L1(0), /*put to 10 later once ground button is a thing */
		L2(21),
		L3(40),
		L4(60),

		BARGE(1.8);

		private double Hight;

		State(double MotorRotations) {
			this.Hight = MotorRotations;
		}

		public double getHight() {
			return Hight;
		}
	}
}
