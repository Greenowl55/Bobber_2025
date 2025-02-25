package frc.robot.NewSubsytems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Coral extends SubsystemBase{

    private final TalonFX motor = new TalonFX(Constants.CORAL_MOTOR_ID);

    public Coral() {
    motor.getConfigurator().apply(new TalonFXConfiguration());
	motor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
