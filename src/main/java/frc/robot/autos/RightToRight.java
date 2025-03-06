package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.coral.Fast;
import frc.robot.commands.elevator.ElevatorPosition;
import frc.robot.commands.tilt.AutoTilt;
import frc.robot.commands.vision.ReefRight;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tilt;

public class RightToRight extends SequentialCommandGroup {
    public RightToRight(CommandSwerveDrivetrain swerve, Elevator elevator, Tilt tilt, Coral coral) throws Exception {

        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile("right to right");

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                new ReefRight(swerve),
                new ElevatorPosition(elevator, Constants.ELEVATOR_L4),
                new AutoTilt(tilt, Constants.FISHHOOK_L4),
                Commands.race(new Fast(coral, Constants.CORAL_FAST), Commands.waitSeconds(Constants.AUTO_TIMEOUT)),
                new AutoTilt(tilt, Constants.FISHHOOK_IN),
                new ElevatorPosition(elevator, Constants.ELEVATOR_INTAKE),
                AutoBuilder.followPath(paths.get(1)));
    }
}
