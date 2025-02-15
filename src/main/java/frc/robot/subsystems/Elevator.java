// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

	TalonFX m_elevator1 = new TalonFX(11);
	TalonFX m_elevator2 = new TalonFX(12);

	public Elevator() {

		var talonFXconfigs = new TalonFXConfiguration();

		// in init function, set slot 0 gains
		var slot0Configs = talonFXconfigs.Slot0;
		slot0Configs.kS = 0.56; // voltage needed to overcome static friction
		slot0Configs.kV = 12.19; // output per unit of target velocity (output/rps)
		slot0Configs.kA = 0.03; // output per unit of target acceleration (output/rps^2)
		slot0Configs.kP =
				1; // output per unit of error in position (output/rotation), An error of 1 rotation
		// results in 2.4 V output
		slot0Configs.kI =
				0; // output per unit of integrated error in position (output/(rotation*s)), no output for
		// integrated error
		slot0Configs.kD =
				0; // output per unit of error in velocity (output/rps), A velocity of 1 rps results in
		// 0.1 V output

		var motionMagicConfigs = talonFXconfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 1000; // velocity in units/100ms
		motionMagicConfigs.MotionMagicAcceleration = 1000; // acceleration in units/100ms^2
		motionMagicConfigs.MotionMagicJerk = 1000; // jerk in units/100ms^3

		// apply motion magic config to slot 0 on Drive1
		m_elevator1.getConfigurator().apply(talonFXconfigs);

		// invert Drive2 and set it to follow Drive1
		m_elevator2.setControl(new Follower(m_elevator1.getDeviceID(), true));
	}

	final PositionVoltage drive1PositionVoltage = new PositionVoltage(0).withSlot(0);

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void setPosition(double position) {
		m_elevator1.setControl(new PositionVoltage(position).withSlot(0));
	}

	public void elevator (double speed){
		m_elevator1.set(speed);
	}
}
