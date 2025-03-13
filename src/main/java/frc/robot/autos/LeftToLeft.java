package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.commands.coral.AutoIntake;
import frc.robot.commands.coral.Fast;
import frc.robot.commands.elevator.ElevatorPosition;
import frc.robot.commands.tilt.AutoTilt;
import frc.robot.commands.vision.ReefLeft;
import frc.robot.commands.vision.ReefRight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Tilt;

public class LeftToLeft extends SequentialCommandGroup {
    public LeftToLeft(CommandSwerveDrivetrain swerve, Elevator elevator, Tilt tilt, Coral coral, Leds leds)throws Exception {	 //EXCEPTION NEEDED???

        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("left to left");             //DOESNT RECOGIZE LEFT TO LEFT I THINK???
        addCommands(
                AutoBuilder.followPath(paths.get(0)).withTimeout(3),
                new ReefLeft(swerve, leds),
                new ElevatorPosition(elevator, "L4", Constants.ELEVATOR_L4),
                new AutoTilt(tilt, Constants.FISHHOOK_L4),
                Commands.race(new Fast(coral, Constants.CORAL_FAST), Commands.waitSeconds(Constants.AUTO_TIMEOUT)),
                new AutoTilt(tilt, Constants.FISHHOOK_IN),
                new ElevatorPosition(elevator, "INTAKE", Constants.ELEVATOR_INTAKE),

                AutoBuilder.followPath(paths.get(1)),
                new AutoIntake(coral, Constants.CORAL_SENSOR, null),

                AutoBuilder.followPath(paths.get(2)),
                new ReefRight(swerve, leds),
                new ElevatorPosition(elevator, "L4", Constants.ELEVATOR_L4),
                new AutoTilt(tilt, Constants.FISHHOOK_L4),
                Commands.race(new Fast(coral, Constants.CORAL_FAST), Commands.waitSeconds(Constants.AUTO_TIMEOUT)),

                AutoBuilder.followPath(paths.get(3)),
                new AutoIntake(coral, Constants.CORAL_SENSOR, null),

                AutoBuilder.followPath(paths.get(4)),
                new ReefLeft(swerve, leds),
                new ElevatorPosition(elevator, "L3", Constants.ELEVATOR_L3),
                Commands.race(new Fast(coral, Constants.CORAL_FAST), Commands.waitSeconds(Constants.AUTO_TIMEOUT)));
                
    }
}



//Hello, my lovely Alex
//                    -Simon