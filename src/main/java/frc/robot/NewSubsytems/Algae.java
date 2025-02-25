package frc.robot.NewSubsytems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.ALGAE_MOTOR_ID);


    public Algae() {
        motor.getConfigurator().apply(new TalonFXConfiguration());
		motor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
