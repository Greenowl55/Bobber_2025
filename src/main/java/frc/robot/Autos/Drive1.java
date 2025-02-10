// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Drive1 extends SequentialCommandGroup {

	public Drive1(CommandSwerveDrivetrain m_swerve) {

		// Optional<Pose2d> pose =
		// PathPlannerAuto.getPathGroupFromAutoFile("Drive").get(0).getStartingHolonomicPose();

		addCommands(
				// Command.runonce(() -> m_swerve.seedfieldrelative(pose)),
				// new PathPlannerAuto("drive")
				AutoBuilder.buildAuto("drive"));
	}
}
