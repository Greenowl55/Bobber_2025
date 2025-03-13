package frc.robot.commands.coral;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Leds;

public class AutoIntake extends Command implements AutoCloseable {
    private double speed;
    private Coral coral;
    private Leds leds;
    private final DigitalInput photoelectricSensor = new DigitalInput(Constants.CORAL_SENSOR);

    public AutoIntake(Coral coral, double speed, Leds leds) {
        super();
        SmartDashboard.putData(this);
        this.coral = coral;
        this.speed = speed;
        this.leds = leds;
        addRequirements(coral, leds);
    }

    private boolean sensorTriggered() {
        return photoelectricSensor.get() == false;
    }

    @Override
    public void initialize() {
        this.coral.setSpeed(this.speed);
    }

    @Override
    public boolean isFinished() {
        return this.sensorTriggered();
    }

    @Override
    public void end(boolean isFinished) {
        this.coral.setSpeed(0);
        leds.greenscroll();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Coral Sensor");
        builder.addBooleanProperty("CoralSensor", () -> this.sensorTriggered(), null);
        builder.publishConstBoolean("Working", true);
    }

    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }
}
