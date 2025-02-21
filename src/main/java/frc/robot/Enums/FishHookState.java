// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Enums;

public class FishHookState {
	public enum Angle {
		IDLE(0.0),
		Intake(0.2),
		L4(0.5),
		Algae(0.75);

		private double angle;

		Angle(double MotorRotations) {
			this.angle = MotorRotations;
		}

		public double getAngle() {
			return angle;
		}
	}

	public enum intake {
		Idle(0),
		INTAKE(0.2);

		private double intake;

		intake(double speed) {
			this.intake = speed;
		}

		public double getSpeed() {
			return intake;
		}
	}
}
