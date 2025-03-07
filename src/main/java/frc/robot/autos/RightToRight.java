package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
import frc.robot.subsystems.Tilt;

public class RightToRight extends SequentialCommandGroup {
    public RightToRight(CommandSwerveDrivetrain swerve, Elevator elevator, Tilt tilt, Coral coral)throws Exception {     //EXCEPTION NEEDED???

        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("right to right");               //DOESNT RECOGIZE LEFT TO LEFT I THINK???

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                new ReefRight(swerve),
                new ElevatorPosition(elevator, "L4", Constants.ELEVATOR_L4),
                new AutoTilt(tilt, Constants.FISHHOOK_L4),
                Commands.race(new Fast(coral, Constants.CORAL_FAST), Commands.waitSeconds(Constants.AUTO_TIMEOUT)),
                new AutoTilt(tilt, Constants.FISHHOOK_IN),
                new ElevatorPosition(elevator, "INTAKE", Constants.ELEVATOR_INTAKE),

                AutoBuilder.followPath(paths.get(1)),
                new AutoIntake(coral, Constants.CORAL_SENSOR),

                AutoBuilder.followPath(paths.get(2)),
                new ReefLeft(swerve),
                new ElevatorPosition(elevator, "L4", Constants.ELEVATOR_L4),
                new AutoTilt(tilt, Constants.FISHHOOK_L4),
                Commands.race(new Fast(coral, Constants.CORAL_FAST), Commands.waitSeconds(Constants.AUTO_TIMEOUT)),

                AutoBuilder.followPath(paths.get(3)),
                new AutoIntake(coral, Constants.CORAL_SENSOR),

                AutoBuilder.followPath(paths.get(4)),
                new ReefRight(swerve),
                new ElevatorPosition(elevator, "L3", Constants.ELEVATOR_L3),
                Commands.race(new Fast(coral, Constants.CORAL_FAST), Commands.waitSeconds(Constants.AUTO_TIMEOUT)));
                
    }
}
