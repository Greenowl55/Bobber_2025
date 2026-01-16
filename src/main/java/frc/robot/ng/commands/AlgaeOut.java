package frc.robot.ng.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ng.subsystems.FishHook;

public class AlgaeOut extends Command {

    private final FishHook fishHook;

    public AlgaeOut(FishHook fishHook) {
        this.fishHook = fishHook;
        this.addRequirements(fishHook);
    }

    @Override
    public void end(boolean interrupted) {
        fishHook.algaeIdle();
    }

    @Override
    public void initialize() {
        fishHook.algaeRollOut();
    }

}
