package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;

public class Led extends Command {
    private Leds m_leds;

    public Led(){
        //addRequirements(m_leds);
    }

    @Override
    public void initialize(){
        m_leds.setRainbow();
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }


}
