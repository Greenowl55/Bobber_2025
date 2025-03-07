package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefCenter extends Command{

        private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private CommandSwerveDrivetrain m_drive;

	private final double rotSetpoint = 0;
	private final double txSetpoint = 0;
	private final double taSetpoint = 4;

	private final double xsetpoint = 1; //TODO
	private final double zsetpoint = 1; //TODO

	// PID Controllers for alignment
	private final PIDController rotationPID;
	private final PIDController strafePID;
	private final PIDController distancePID;

	private double xoffset;
	private double zoffset;
	private double rotoffset;
	//public static Pose2d tPose2d(double[] inData);

	public ReefCenter(CommandSwerveDrivetrain drive_subsystem) {
		addRequirements(drive_subsystem);
		m_drive = drive_subsystem;

		// Configure PID controllers with gains
		rotationPID = new PIDController(0.025, 0, 0 );
		strafePID = new PIDController(0.1, 0.01, 0.002);
		distancePID = new PIDController(0.1, 0.01, 0.002);
		addRequirements(drive_subsystem);

		// Set tolerances for when we consider ourselv+es "aligned"
		rotationPID.setTolerance(1);
		strafePID.setTolerance(1);
		distancePID.setTolerance(1);
	}

	@Override
	public void initialize() {
		rotationPID.reset();
		strafePID.reset();
		distancePID.reset();
		
			rotationPID.setSetpoint(rotSetpoint);
			strafePID.setSetpoint(txSetpoint);
			distancePID.setSetpoint(taSetpoint);
	}

	@Override
	public void execute() {
		double tx = LimelightHelpers.getTX(Constants.LIMELIGHT4_NAME);// Horizontal offset from crosshair to target in degrees
		double ty = LimelightHelpers.getTY(Constants.LIMELIGHT4_NAME);// Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA(Constants.LIMELIGHT4_NAME);// Target area (0% of image to 100% of image)
		double id = LimelightHelpers.getFiducialID(Constants.LIMELIGHT4_NAME);
        double txnc = LimelightHelpers.getTXNC(Constants.LIMELIGHT4_NAME);  // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC(Constants.LIMELIGHT4_NAME);  // Vertical offset from principal pixel/point to target in degrees
		Pose3d pose = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LIMELIGHT4_NAME);
		double rot = Math.toDegrees(pose.getRotation().getY());
		double[] fieldpose = LimelightHelpers.getBotPose_TargetSpace(Constants.LIMELIGHT4_NAME);


		boolean tagFound = false;
		for (int tag : Constants.REEF_TAGS) {
			if (id == tag) {
				tagFound = true;
				break;
			}
		}
		if (tagFound == true) { // Calculate control outputs
			double botX = fieldpose[0]; //offset from tag in meters
			double botZ = fieldpose[2]; //distance from tag in meters
			double yawtotag = fieldpose[5]; // angle to tag in degrees

			double fieldyaw = m_drive.getState().Pose.getRotation().getDegrees(); // robots angle on field
			
			double targetyaw = fieldyaw - yawtotag + rotSetpoint; //sets robot angle to be inline with tag

			double xoffset = xsetpoint - botX;
			double zoffset = zsetpoint - botZ;
			double rotoffset = rotSetpoint - yawtotag;
			
			double rotationOutput = rotationPID.calculate(rotoffset);
			double strafeOutput = strafePID.calculate(xoffset);
			double forwardOutput = distancePID.calculate(zoffset);

			//limit speed
			rotationOutput = Math.max(-Math.PI, Math.min(Math.PI, rotationOutput));
			strafeOutput = Math.max(-1, Math.min(1, strafeOutput));
			forwardOutput = Math.max(-1, Math.min(1, forwardOutput));

			// Apply combined movement
			m_drive.setControl(
					drive
							.withVelocityX(forwardOutput) // Forward/backward
							.withVelocityY(strafeOutput) // Left/right
							.withRotationalRate(rotationOutput) ); // Rotation
		} else {
			// If we don't see the correct tag, stop moving
			m_drive.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
		}

		
	}

	// @Override
	// public void end(boolean interrupted) {
	// 	m_drive.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
	// }

	@Override
	public boolean isFinished() {
		// Command finishes when we're aligned with the target
		return rotationPID.atSetpoint() && strafePID.atSetpoint() && distancePID.atSetpoint();
	}

	@Override
	public boolean runsWhenDisabled() {
		return false;
	}
    
        @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("reef offsets"); //to get usable values make setpoints 0
        builder.publishConstDouble("rotation offset", xoffset);
		builder.publishConstDouble("strafe offset", zoffset);
		builder.publishConstDouble("rotation", rotoffset);
    }

}
