// Copyright (c) 2024 - 2025 : FRC 3102 : Tech-No-Tigers
// https://www.tnt3102.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.platform.can.AutocacheState;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.autos.*;
import frc.robot.commands.Climb.*;
import frc.robot.commands.algae.*;
import frc.robot.commands.coral.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.tilt.*;
import frc.robot.commands.vision.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {

	Elevator m_elevator = new Elevator();
	Coral m_coral = new Coral();
	Algae m_algae = new Algae();
	Tilt m_tilt = new Tilt();
	Climber m_climber = new Climber();

	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75)
			.in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.05)
			.withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
			.withDriveRequestType(
					DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final Telemetry logger = new Telemetry(MaxSpeed);

	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandGenericHID coDriverController = new CommandGenericHID(1);
	private final Joystick codriverJoystick = new Joystick(1);

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(
						() -> drive
								.withVelocityX(
										-driverController.getLeftY()
												* MaxSpeed) // Drive forward with negative Y (forward)
								.withVelocityY(
										-driverController.getLeftX()
												* MaxSpeed) // Drive left with negative X (left)
								.withRotationalRate(
										-driverController.getRightX()
												* MaxAngularRate) // Drive counterclockwise with negative X (left)
				));

		// TODO Find button for:
		// driverController.a().toggleOnTrue(drivetrain.applyRequest(() ->
		// brake));
		// joystick.b().whileTrue(drivetrain.applyRequest(() ->
		// point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
		// -joystick.getLeftX()))
		// ));

		driverController
				.pov(0)
				.whileTrue(
						drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
		driverController
				.pov(180)
				.whileTrue(
						drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		driverController
				.back()
				.and(driverController.y())
				.whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		driverController
				.back()
				.and(driverController.x())
				.whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		driverController
				.start()
				.and(driverController.y())
				.whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		driverController
				.start()
				.and(driverController.x())
				.whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on start button press
		driverController.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		drivetrain.registerTelemetry(logger::telemeterize);

	// Driver Buttons

		// Elevator positions
		driverController.button(8).onTrue(new ElevatorPosition(m_elevator, Constants.ELEVATOR_BOTTOM));
		driverController.a().onTrue(new ElevatorPosition(m_elevator, Constants.ELEVATOR_INTAKE));
		driverController.b().onTrue(new ElevatorPosition(m_elevator, Constants.ELEVATOR_L2));
		driverController.x().onTrue(new ElevatorPosition(m_elevator, Constants.ELEVATOR_L3));
		driverController.y().onTrue(new ElevatorPosition(m_elevator, Constants.ELEVATOR_L4));
		// driverController.rightBumper().whileTrue(new ManualControl(m_elevator, 0.2));
		// driverController.leftBumper().whileTrue(new ManualControl(m_elevator, -0.2));
		// m_elevator.setDefaultCommand(m_elevator.runOnce(() -> {m_elevator.elevator(0);}));

		// Reef positions
		driverController.rightTrigger().whileTrue(new ReefCenter(drivetrain));
		driverController.rightBumper().whileTrue(new ReefRight(drivetrain));
		driverController.leftBumper().whileTrue(new ReefLeft(drivetrain));
		driverController.leftTrigger().whileTrue(new VisonOutput());

	// Co-Driver Buttons

		// game piece control
		coDriverController.button(Constants.CODRIVER_2).onTrue(new AutoIntake(m_coral, Constants.CORAL_SLOW).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
		coDriverController.button(Constants.CODRIVER_1).whileTrue(new Fast(m_coral, Constants.CORAL_FAST).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
		coDriverController.button(Constants.CODRIVER_4).whileTrue(new RollIn(m_algae, Constants.ALGAE_IN).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
		coDriverController.button(Constants.CODRIVER_3).whileTrue(new Rollout(m_algae, Constants.ALGAE_OUT).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

		//tilt control
		coDriverController.button(Constants.CODRIVER_7).whileTrue(new Tilt_Position(m_tilt, Constants.FISHHOOK_GROUND));
		coDriverController.button(Constants.CODRIVER_8).whileTrue(new Tilt_Position(m_tilt, Constants.FISHHOOK_L4));
		coDriverController.button(Constants.CODRIVER_9).whileTrue(new Tilt_Position(m_tilt, Constants.FISHHOOK_IN));

		// climber control
		coDriverController.button(Constants.CODRIVER_5).whileTrue(new Climber_In(m_climber, 0.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
		coDriverController.button(Constants.CODRIVER_6).whileTrue(new Climber_Out(m_climber, -0.2).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
		//m_tilt.setDefaultCommand(m_tilt.run(() -> m_tilt.setspeed(codriverJoystick.getY()*-0.1)));
	}

		/* Path follower */
		private final SendableChooser<Command> autoChooser;

		public RobotContainer() {
			autoChooser = AutoBuilder.buildAutoChooser("Tests");
			SmartDashboard.putData("Auto Mode", autoChooser);
	
			autoChooser.setDefaultOption("Do Nothing", new PrintCommand("Do Nothing"));
			autoChooser.addOption("Drive", new Drive1(drivetrain));
			//autoChooser.addOption("forward", drivetrain.getAutoPath("Drive"));
	
			configureBindings();
	
		}
	
	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.getSelected();
	}
}
