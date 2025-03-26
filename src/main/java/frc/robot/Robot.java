// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.TargetDebug;

public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;

	private final RobotContainer m_robotContainer;

	private final boolean kUseLimelight = false;

	public Robot() {
		Logger.recordMetadata("ProjectName", "Bobber"); // Set a metadata value

		switch (Constants.currentMode) {
			case REAL:
			  Logger.addDataReceiver(new WPILOGWriter());
			  Logger.addDataReceiver(new NT4Publisher());
			  break;
	  
			case SIM:
			  Logger.addDataReceiver(new NT4Publisher());
			  break;
	  
			case REPLAY:
			  setUseTiming(false);
			  String logPath = LogFileUtil.findReplayLog();
			  Logger.setReplaySource(new WPILOGReader(logPath));
			  Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
			  break;
		  }

		Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
		
		
		m_robotContainer = new RobotContainer();
	
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		Logger.recordOutput("RobotPose", new Pose2d());
		Logger.recordOutput("ZeroedComponent", new Pose3d[] {new Pose3d()}) ;
		Logger.recordOutput("FinalComponentPoses", new Pose3d[] {
			new Pose3d(0, 0, 0, new Rotation3d(0, Math.sin(Timer.getTimestamp()) - 1, 0))
		});
		/*
		 * This example of adding Limelight is very simple and may not be sufficient for
		 * on-field use.
		 * Users typically need to provide a standard deviation that scales with the
		 * distance to target
		 * and changes with number of tags available.
		 *
		 * This example is sufficient to show that vision integration is possible,
		 * though exact implementation
		 * of how to use vision should be tuned per-robot and to the team's
		 * specification.
		 */
		if (kUseLimelight) {

			// Get drivetrain information
			var driveState = m_robotContainer.drivetrain.getState();
			double headingDeg = driveState.Pose.getRotation().getDegrees();
			double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

			LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
			var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
			if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
				m_robotContainer.drivetrain.addVisionMeasurement(
						llMeasurement.pose, llMeasurement.timestampSeconds);
			}
		}
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		CommandScheduler.getInstance().registerSubsystem(new TargetDebug(Constants.LIMELIGHT4_NAME));
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}
