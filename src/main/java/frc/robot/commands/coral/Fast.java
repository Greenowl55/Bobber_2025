package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class Fast extends Command {

    private double speed;
    private Coral coral;

    public Fast(Coral coral, double speed) {
        this.coral = coral;
        this.speed = speed;
        addRequirements(coral);
    }

    @Override
    public void initialize() {
        this.coral.setSpeed(this.speed);
    }

    @Override
    public void end(boolean interrupted) {
        this.coral.setSpeed(0);
    }
}
