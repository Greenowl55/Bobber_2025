package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class AutoIntake extends Command {
    private double speed;
    private Coral coral;
    private final DigitalInput photoelectricSensor = new DigitalInput(Constants.CORAL_SENSOR);

    public AutoIntake(Coral coral, double speed) {
        this.coral = coral;
        this.speed = speed;
        addRequirements(coral);
    }

    @Override
    public void initialize() {
        this.coral.setSpeed(this.speed);
    }

    @Override
    public boolean isFinished() {
        return (photoelectricSensor.get());
    }

    @Override
    public void end(boolean isFinished) {
        this.coral.setSpeed(0);
    }
}
