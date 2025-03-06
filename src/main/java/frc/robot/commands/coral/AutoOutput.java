package frc.robot.commands.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Coral;

public class AutoOutput extends SequentialCommandGroup {

    private double speed;
    private Coral coral;
    private int millis;

    public AutoOutput(Coral coral, double speed, double seconds) {
        this.coral = coral;
        this.speed = speed;
        addRequirements(coral);

        addCommands(Commands.race(new Fast(coral, speed), Commands.waitSeconds(seconds)));
    }

}
