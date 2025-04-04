package frc.robot.commands.algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;

public class RollIn extends Command {
    private double speed;
    private Algae algae;
    private Coral coral;

    public RollIn(Algae algae, Coral coral, double speed) {
        this.speed = speed;
        this.algae = algae;
        this.coral = coral;
        addRequirements(coral, algae);
    }

    @Override
    public void initialize() {
        this.algae.setSpeed(this.speed);
        this.coral.setSpeed(Constants.CORAL_SLOW);
    }

    @Override
    public void end(boolean interrupted) {
        this.algae.setSpeed(0);
        this.coral.setSpeed(0);
    }

}
