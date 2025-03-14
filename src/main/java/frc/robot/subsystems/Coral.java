package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Coral extends SubsystemBase {

    private final TalonFX motor = new TalonFX(Constants.CORAL_MOTOR_ID);
    private final DigitalInput photoelectricSensor = new DigitalInput(Constants.CORAL_SENSOR);


    public Coral() {
        super();
        SmartDashboard.putData(this);
        //SendableRegistry.addLW(this, "coral");
        motor.getConfigurator().apply(new TalonFXConfiguration());
        motor.setNeutralMode(NeutralModeValue.Brake);
        
    }

    public boolean sensorTriggered() {
        return photoelectricSensor.get() == false;
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("CoralConnected", () -> motor.isConnected(), null);
        builder.addBooleanProperty("CoralAlive", () -> motor.isAlive(), null);
        builder.publishConstBoolean("Working", true);
    }


}
