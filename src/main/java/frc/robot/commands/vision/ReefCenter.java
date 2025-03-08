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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ReefCenter extends Command{

        private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private CommandSwerveDrivetrain m_drive;

	private double rotSetpoint = 0;
	private final double txSetpoint = 0;
	private final double taSetpoint = 4;

	private double xsetpoint = 1; //TODO
	private double zsetpoint = 1; //TODO
	private boolean tagVisible = false;

	// PID Controllers for alignment
	private final PIDController rotationPID;
	private final PIDController strafePID;
	private final PIDController distancePID;

	private double rotationP = 0.025;
	private double rotationI = 0;
	private double rotationD = 0;

	private double strafeP = 0.1;
	private double strafeI = 0.01;
	private double strafeD = 0.002;

	private double distanceP = 0.1;
	private double distanceI = 0.01;
	private double distanceD = 0.002;


	private double botX = 0;
	private double botZ = 0;
	private double botYaw = 0;
	//public static Pose2d tPose2d(double[] inData);

	public ReefCenter(CommandSwerveDrivetrain drive_subsystem) {
		addRequirements(drive_subsystem);
		m_drive = drive_subsystem;

		// Configure PID controllers with gains
		rotationPID = new PIDController(rotationP, rotationI, rotationD );
		strafePID = new PIDController(strafeP, strafeI, strafeD);
		distancePID = new PIDController(distanceP, distanceI, distanceD);
		SmartDashboard.putData(this);

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
		tagVisible = false;
		for (int tag : Constants.REEF_TAGS) {
			if (id == tag) {
				tagFound = true;
				break;
			}
		}
		if (tagFound == true) { // Calculate control outputs
			this.tagVisible = true;
			botX = fieldpose[0]; //offset from tag in meters
			botZ = fieldpose[2]; //distance from tag in meters
			double yawtotag = fieldpose[5]; // angle to tag in degrees
			this.botYaw = yawtotag;
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
        builder.addBooleanProperty("tagVisible", () -> this.tagVisible , null);
		builder.addDoubleProperty("xOffset", () -> this.xsetpoint, (val) -> 
		{
			this.xsetpoint = val;
			this.strafePID.setSetpoint(xsetpoint);
		});
        builder.addDoubleProperty("zOffset", () -> this.zsetpoint, (val) ->{ 
			this.zsetpoint = val;
			this.distancePID.setSetpoint(zsetpoint);
		});
        builder.addDoubleProperty("rotOffset", () -> this.rotSetpoint, (val) -> {
			this.rotSetpoint = val;
			this.rotationPID.setSetpoint(rotSetpoint);
		});
		builder.addDoubleProperty("botX", () -> this.botX, null);
		builder.addDoubleProperty("botZ", () -> this.botZ, null);
		builder.addDoubleProperty("botYaw", () -> this.botYaw, null);
		builder.addBooleanProperty("inPosition", () -> {
			return rotationPID.atSetpoint() && strafePID.atSetpoint() && distancePID.atSetpoint();
		}, null);
		builder.addDoubleProperty("rotationP", () -> this.rotationP, (val) -> {
			this.rotationP = val;
			rotationPID.setP(this.rotationP);
		});
		builder.addDoubleProperty("rotationI", () -> this.rotationI, (val) -> {
			this.rotationI = val;
			rotationPID.setI(this.rotationI);
		});
		builder.addDoubleProperty("rotationD", () -> this.rotationD, (val) -> {
			this.rotationD = val;
			rotationPID.setD(this.rotationD);
		});

		builder.addDoubleProperty("strafeP", () -> this.strafeP, (val) -> {
			this.strafeP = val;
			strafePID.setP(this.strafeP);
		});
		builder.addDoubleProperty("strafeI", () -> this.strafeI, (val) -> {
			this.strafeI = val;
			strafePID.setI(this.strafeI);
		});
		builder.addDoubleProperty("strafeD", () -> this.strafeD, (val) -> {
			this.strafeD = val;
			strafePID.setD(this.strafeD);
		});

		builder.addDoubleProperty("distanceP", () -> this.distanceP, (val) -> {
			this.distanceP = val;
			distancePID.setP(this.distanceP);
		});
		builder.addDoubleProperty("distanceI", () -> this.distanceI, (val) -> {
			this.distanceI = val;
			distancePID.setI(this.distanceI);
		});
		builder.addDoubleProperty("distanceD", () -> this.distanceD, (val) -> {
			this.distanceD = val;
			distancePID.setD(this.distanceD);
		});
    }

}
