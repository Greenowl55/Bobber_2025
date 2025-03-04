package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Coral extends SubsystemBase {

    private final TalonFX motor = new TalonFX(Constants.CORAL_MOTOR_ID);

    public Coral() {
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("CoralConnected", () -> motor.isConnected(), null);
        builder.addBooleanProperty("CoralAlive", () -> motor.isAlive(), null);
    }


}
