package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.ALGAE_MOTOR_ID);

    public Algae() {

        var limitConfigs = new CurrentLimitsConfigs();

        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.setNeutralMode(NeutralModeValue.Brake);
        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 50; //TODO tune this
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimit = 30; //TODO tune this
        limitConfigs.SupplyCurrentLimitEnable = true;
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("AlgaeConnected", () -> motor.isConnected(), null);
        builder.addBooleanProperty("AlgaeAlive", () -> motor.isAlive(), null);
    }
}
