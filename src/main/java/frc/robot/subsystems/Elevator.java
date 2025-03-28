// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Elevator extends SubsystemBase {

	TalonFX m_elevator1 = new TalonFX(Constants.ELEVATOR_LEADER);
	TalonFX m_elevator2 = new TalonFX(Constants.ELEVATOR_FOLLOWER);

	public Elevator() {
		SmartDashboard.putData(this);
		var talonFXconfigs = new TalonFXConfiguration();

		// in init function, set slot 0 gains
		var slot0Configs = talonFXconfigs.Slot0;
		slot0Configs.kS = 0.56; // voltage needed to overcome static friction
		slot0Configs.kV = 0.1219; // output per unit of target velocity (output/rps)
		slot0Configs.kA = 0.02; // output per unit of target acceleration (output/rps^2)
		slot0Configs.kP = 5; // output per unit of error in position (output/rotation)
		slot0Configs.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
		slot0Configs.kD = 0; // output per unit of error in velocity (output/rps)

		var motionMagicConfigs = talonFXconfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 275; // velocity in units/100ms
		motionMagicConfigs.MotionMagicAcceleration = 250; // acceleration in units/100ms^2
		motionMagicConfigs.MotionMagicJerk = 500; // jerk in units/100ms^3

		// apply motion magic config to slot 0 on Drive1
		m_elevator1.getConfigurator().apply(talonFXconfigs);

		m_elevator1.setNeutralMode(NeutralModeValue.Brake);
		m_elevator2.setNeutralMode(NeutralModeValue.Brake);

		// invert Drive2 and set it to follow Drive1
		m_elevator2.setControl(new Follower(m_elevator1.getDeviceID(), true));
	}

	// final PositionVoltage drive1PositionVoltage = new
	// PositionVoltage(0).withSlot(0);
	final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	public void setPosition(double position) {
		// m_elevator1.setControl(new PositionVoltage(position).withSlot(0));
		m_elevator1.setControl(m_request.withPosition(position));
	}

	public void elevator(double speed) {
		m_elevator1.set(speed);
	}

	public boolean isHomed = false;

	Trigger currentOverZeroThreshold = new Trigger(() -> m_elevator1.getSupplyCurrent().getValueAsDouble() > 10)
			.debounce(0.2);

	public Command zeroElevator() {

		return this.runOnce(
				() -> {
					m_elevator1.setVoltage(-3);
					isHomed = false;
				})
				.until(currentOverZeroThreshold)
				.andThen(
						this.runOnce(
								() -> {
									m_elevator1.setControl(new NeutralOut());
									m_elevator1.setPosition(0);
									isHomed = true;
								}));
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addBooleanProperty("ElevatorLeaderConnected", () -> m_elevator1.isConnected(), null);
		builder.addBooleanProperty("ElevatorLeaderAlive", () -> m_elevator1.isAlive(), null);
		builder.addBooleanProperty("ElevatorFollowerConnected", () -> m_elevator2.isConnected(), null);
		builder.addBooleanProperty("ElevatorFollowerAlive", () -> m_elevator2.isAlive(), null);
		builder.addDoubleProperty("ElevatorPosition", () -> m_elevator1.getPosition().getValueAsDouble(), null);
	}

	public double getPosition() {
		return m_elevator1.getPosition().getValueAsDouble();
	}

}
