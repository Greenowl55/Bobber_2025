// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefLeft extends Command {
	private final SwerveRequest.RobotCentric drive =
			new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private CommandSwerveDrivetrain m_drive;
	private int[] m_tag;
	private int[] ReefTags;

	// PID Controllers for alignment
	private final PIDController rotationPID;
	private final PIDController strafePID;
	private final PIDController distancePID;

	public ReefLeft(CommandSwerveDrivetrain drive_subsystem) {
		addRequirements(drive_subsystem);
		m_tag = ReefTags;
		m_drive = drive_subsystem;

		// Configure PID controllers with gains
		rotationPID = new PIDController(0.015, 0, 0);
		strafePID = new PIDController(0.015, 0, 0);
		distancePID = new PIDController(0.1, 0, 0);
		addRequirements(drive_subsystem);

		ReefTags = new int[12];
		ReefTags[0] = 6;
		ReefTags[1] = 7;
		ReefTags[2] = 8;
		ReefTags[3] = 9;
		ReefTags[4] = 10;
		ReefTags[5] = 11;
		ReefTags[6] = 17;
		ReefTags[7] = 18;
		ReefTags[8] = 19;
		ReefTags[9] = 20;
		ReefTags[10] = 21;
		ReefTags[11] = 22;

		// Set tolerances for when we consider ourselv+es "aligned"
		rotationPID.setTolerance(1.0);
		strafePID.setTolerance(1.0);
		distancePID.setTolerance(0.5);
	}

	@Override
	public void initialize() {
		rotationPID.reset();
		strafePID.reset();
		distancePID.reset();
	}

	@Override
	public void execute() {
		double tx = LimelightHelpers.getTX("ll4");
		double ty = LimelightHelpers.getTY("ll4");
		double id = LimelightHelpers.getFiducialID("ll4");

		boolean tagFound = false;
		for (int tag : m_tag) {
			if (id == tag) {
				tagFound = true;
				break;
			}
		}
		if (tagFound == true) { // Calculate control outputs
			double rotationOutput = rotationPID.calculate(tx, 0) * 1.5 * Math.PI;
			double strafeOutput = strafePID.calculate(tx, -5);
			double forwardOutput = distancePID.calculate(ty, 0);

			// Apply combined movement
			m_drive.setControl(
					drive
							.withVelocityX(forwardOutput) // Forward/backward
							.withVelocityY(strafeOutput) // Left/right
							.withRotationalRate(rotationOutput * -1)); // Rotation
		} else {
			// If we don't see the correct tag, stop moving
			m_drive.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
		}
	}

	@Override
	public void end(boolean interrupted) {
		m_drive.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
	}

	@Override
	public boolean isFinished() {
		// Command finishes when we're aligned with the target
		return rotationPID.atSetpoint() && strafePID.atSetpoint() && distancePID.atSetpoint();
	}

	@Override
	public boolean runsWhenDisabled() {
		return false;
	}
}
