// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;

public class ReefLeft extends Command {
	private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private CommandSwerveDrivetrain m_drive;

	// PID Controllers for alignment
	private final PIDController rotationPID;
	private final PIDController strafePID;
	private final PIDController distancePID;

	public ReefLeft(CommandSwerveDrivetrain drive_subsystem) {
		addRequirements(drive_subsystem);
		m_drive = drive_subsystem;

		// Configure PID controllers with gains
		rotationPID = new PIDController(0.05, 0, 0);
		strafePID = new PIDController(0.05, 0, 0);
		distancePID = new PIDController(0.1, 0, 0);
		addRequirements(drive_subsystem);

		// Set tolerances for when we consider ourselv+es "aligned"
		rotationPID.setTolerance(1);
		strafePID.setTolerance(1.0);
		distancePID.setTolerance(1);
	}

	@Override
	public void initialize() {
		rotationPID.reset();
		strafePID.reset();
		distancePID.reset();
	}

	@Override
	public void execute() {
		double tx = LimelightHelpers.getTX("limelight");
		double ty = LimelightHelpers.getTY("limelight");
		double id = LimelightHelpers.getFiducialID("limelight");

		boolean tagFound = false;
		for (int tag : Constants.REEF_TAGS) {
			if (id == tag) {
				tagFound = true;
				break;
			}
		}
		if (tagFound == true) { // Calculate control outputs
			double rotationOutput = rotationPID.calculate(tx, 9.26) /*  1.5 * Math.PI*/;
			double strafeOutput = strafePID.calculate(tx, 9.26);
			double forwardOutput = distancePID.calculate(ty, 0.39);

			// Apply combined movement
			m_drive.setControl(
					drive
							.withVelocityX(forwardOutput) // Forward/backward
							.withVelocityY(strafeOutput) // Left/right
							.withRotationalRate(rotationOutput)); // Rotation
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
