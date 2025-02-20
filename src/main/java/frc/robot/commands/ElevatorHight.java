// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

public class ElevatorHight {
	public enum State {
		BOTTOM(0.0),
		L1(10),
		L2(15),
		L3(25),
		L4(35),

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
