// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Enums.FishHookState;

public class Fish_Hook extends SubsystemBase {

	private final TalonFX m_coral = new TalonFX(10);
	private final TalonFX m_algae = new TalonFX(9);
	private final TalonFX m_tilt = new TalonFX(8);
	private FishHookState.intake m_intake_state = FishHookState.intake.Idle;

	private final DigitalInput photoelectricSensor = new DigitalInput(0);
	private final DigitalOutput PES = new DigitalOutput(1);

	public Fish_Hook() {

		var talonFXconfigs = new TalonFXConfiguration();

		// in init function, set slot 0 gains
		var slot0Configs = talonFXconfigs.Slot0;
		slot0Configs.kS = 0.2; // voltage needed to overcome static friction
		slot0Configs.kV = 0.1; // output per unit of target velocity (output/rps)
		slot0Configs.kA = 0.1; // output per unit of target acceleration (output/rps^2)
		slot0Configs.kP =
				2.4; // output per unit of error in position (output/rotation), An error of 1 rotation
		// results in 2.4 V output
		slot0Configs.kI =
				0; // output per unit of integrated error in position (output/(rotation*s)), no output for
		// integrated error
		slot0Configs.kD =
				0.1; // output per unit of error in velocity (output/rps), A velocity of 1 rps results in
		// 0.1 V output

		var motionMagicConfigs = talonFXconfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 1000; // velocity in units/100ms
		motionMagicConfigs.MotionMagicAcceleration = 1000; // acceleration in units/100ms^2
		motionMagicConfigs.MotionMagicJerk = 1000; // jerk in units/100ms^3

		m_tilt.getConfigurator().apply(talonFXconfigs);

		m_coral.getConfigurator().apply(new TalonFXConfiguration());
		m_coral.setNeutralMode(NeutralModeValue.Brake);

		m_algae.getConfigurator().apply(new TalonFXConfiguration());
		m_algae.setNeutralMode(NeutralModeValue.Brake);

		m_tilt.getConfigurator().apply(new TalonFXConfiguration());
		m_tilt.setNeutralMode(NeutralModeValue.Brake);
	}

	final PositionVoltage drive1PositionVoltage = new PositionVoltage(0).withSlot(0);

	@Override
	public void periodic() {
		// periodic code here
		if (m_intake_state.equals(FishHookState.intake.INTAKE)) {
			if (photoelectricSensor.get()) {
				this.coral(0);
				m_intake_state = FishHookState.intake.Idle;
			}
		}
	}

	public boolean autoIntakeRunning() {
		return m_intake_state.equals(FishHookState.intake.INTAKE);
	}

	public void startAutoIntake() {
		m_intake_state = FishHookState.intake.INTAKE;
	}

	@Override
	public void simulationPeriodic() {}

	public void coral(double speed) {
		m_coral.set(speed);
	}

	public void algae(double speed) {
		m_algae.set(speed);
	}

	public void setposition(double position) {
		m_tilt.setControl(new PositionVoltage(position).withSlot(0));
	}

	public void tilt(Double speed) {
		m_tilt.set(speed);
	}

	public boolean isHomed = false;

	Trigger currentOverZeroThreshold =
			new Trigger(() -> m_tilt.getSupplyCurrent().getValueAsDouble() > 10).debounce(0.2);

	public Command zeroFishhook() {

		return this.run(
						() -> {
							m_tilt.setVoltage(-3);
							isHomed = false;
						})
				.until(currentOverZeroThreshold)
				.andThen(
						this.runOnce(
								() -> {
									m_tilt.setControl(new NeutralOut());
									m_tilt.setPosition(0);
									isHomed = true;
								}));
	}
}
