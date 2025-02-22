// // Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// // https://www.tnt3102.org

// // Use of this source code is governed by an MIT-style
// // license that can be found in the LICENSE file at
// // the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.ClimberState.State;
// import frc.robot.subsystems.Climber;

// public class CMD_ClimberState extends SequentialCommandGroup {

// 	public CMD_ClimberState(State newScoringState, Climber Climber) {

// 		addRequirements(Climber);

// 		addCommands(Climber.runOnce(() -> Climber.setPosition(newScoringState.getAngle())));
// 	}
// }
