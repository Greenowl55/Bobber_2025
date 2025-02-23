package frc.robot.ng.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ng.subsystems.FishHook;

public class CoralAuto extends Command {

    private final FishHook fishHook;

    public CoralAuto(FishHook fishhook) {
        fishHook = fishhook;
        addRequirements(fishHook);
    }

    @Override
    public void initialize() {
        fishHook.intakeAuto();
    }
}
