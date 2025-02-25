package frc.robot.NewCommands.coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.NewSubsytems.Coral;

public class AutoIntake extends Command {
    private double speed;
    private Coral coral;
    private final DigitalInput photoelectricSensor = new DigitalInput(Constants.Coral_Sensor);

    public AutoIntake(Coral coral, double speed) {
        this.coral = coral;
        this.speed = speed;
        addRequirements(coral);
    }
    
    @Override
    public void initialize() {
        //this.coral.setSpeed(this.speed);
    }

    @Override 
    public void execute(){
        if (photoelectricSensor.get()){
            coral.setSpeed(speed);
        }
        else{
            coral.setSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.coral.setSpeed(0);
    }
}
    






