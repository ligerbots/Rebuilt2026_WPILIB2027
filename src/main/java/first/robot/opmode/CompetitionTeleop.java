// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.StartEndCommand;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.driverstation.XboxController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import first.robot.Robot;
import first.robot.subsystems.CommandSwerveDrivetrain;
import first.robot.subsystems.intake.Intake;
import first.robot.subsystems.shooter.Shooter;
import first.robot.subsystems.shooter.Shooter.ShotType;

@Teleop
public class CompetitionTeleop extends PeriodicOpMode {
    protected static final double JOYSTICK_DEADBAND = 0.05;

    // Setting up bindings for necessary control of the swerve drive platform
    protected final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // set the swerve wheels in an X pattern
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private SlewRateLimiter m_xLimiter = null;
    private SlewRateLimiter m_yLimiter = null;
    private SlewRateLimiter m_rotationLimiter = null;

    private final double m_maxSpeed;
    private final double m_maxAngularSpeed;

    // This is "protected" so that the subclasses can access it
    protected final Robot m_robot;

    /** The Robot instance is passed into the opmode via the constructor. */
    public CompetitionTeleop(Robot robot) {
        this(robot, 0, 0, 1.0);
    }

    public CompetitionTeleop(Robot robot, double driveMaxSlewRate, double turnMaxSlewRate, double speedScale) {
        m_robot = robot;

        // Extra parameters for more controlled driving
        m_maxSpeed = Robot.MAX_SPEED * speedScale;
        m_maxAngularSpeed = Robot.MAX_ANGULAR_RATE * speedScale;

        if (driveMaxSlewRate > 0.0) {
            m_xLimiter = new SlewRateLimiter(driveMaxSlewRate);
            m_yLimiter = new SlewRateLimiter(driveMaxSlewRate);
        }
        if (turnMaxSlewRate > 0.0) {
            m_rotationLimiter = new SlewRateLimiter(turnMaxSlewRate);
        }

        driveBindings();
        mainBindings();
    }

    // The binding configuration is broken into a couple of routines so that
    // different pieces can be overridden
    void driveBindings() {
        // for convenience here, since they are used so frequently below
        CommandXboxController driverController = m_robot.getDriverController();
        CommandSwerveDrivetrain drivetrain = m_robot.getDrivetrain();

        // use the built-in deadband feature, but set the scale
        setDeadband(driverController.getController(), JOYSTICK_DEADBAND);

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> m_driveRequest
                        .withVelocityX(-conditionAxis(driverController.getLeftY(), m_xLimiter) * m_maxSpeed)
                        .withVelocityY(-conditionAxis(driverController.getLeftX(), m_yLimiter) * m_maxSpeed)
                        .withRotationalRate(-conditionAxis(driverController.getRightX(), m_rotationLimiter) * m_maxAngularSpeed)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    }

    void mainBindings() {
        // for convenience here, since they are used so frequently below
        CommandXboxController driverController = m_robot.getDriverController();
        CommandGenericHID farmController = m_robot.getFarmController();
        Intake intake = m_robot.getIntake();
        Shooter shooter = m_robot.getShooter();

        // enable/disable brake mode on the pivot when the robot is disabled
        RobotModeTriggers.disabled().onFalse(new InstantCommand(() -> intake.getPivot().setBrakeMode(false)));
        RobotModeTriggers.disabled().onTrue(
                new InstantCommand(() -> intake.getPivot().setBrakeMode(true)).ignoringDisable(true));

        // Just shoot
        driverController.rightTrigger().whileTrue(m_robot.shootCommand(ShotType.AUTO));

        // shoot while intaking
        driverController.rightBumper().whileTrue(m_robot.shootCommand(ShotType.AUTO));
        driverController.rightBumper().onTrue(intake.getPivot().deployCommand());
        driverController.rightBumper().whileTrue(
                new StartEndCommand(intake.getRoller()::intake, intake.getRoller()::stop, intake.getRoller()));

        // Deploy and run the intake (intake will stay out)
        driverController.leftTrigger().onTrue(intake.getPivot().deployCommand());
        driverController.leftTrigger().whileTrue(
                new StartEndCommand(intake.getRoller()::fastIntake, intake.getRoller()::stop,
                        intake.getRoller()));
        // .alongWith(new StartEndCommand(m_robot.hopper::intake, m_robot.hopper::stop, m_m_robot.hopper)));

        // Stow the intake
        driverController.leftBumper().onTrue(intake.stowCommand());

        // lock wheels
        driverController.menu().whileTrue(m_robot.getDrivetrain().applyRequest(() -> m_brakeRequest));

        // Unjam
        farmController.button(21).whileTrue(unjamCommand());
        driverController.a().whileTrue(unjamHopperCommand());
        driverController.b().whileTrue(unjamHopperCommand());
        driverController.y().whileTrue(unjamHopperCommand());
        driverController.x().whileTrue(unjamHopperCommand());

        // fixed shots - distance in inches, plus ROBOT angle of turret
        // ladder - robot against the outside of the ladder, intake to the left of the driver
        farmController.button(11).whileTrue(m_robot.shootCommand(130.0, Rotation2d.CCW_90DEG));

        // corner shot
        farmController.button(13).whileTrue(m_robot.shootCommand(210.0, Rotation2d.PI));

        farmController.button(15).whileTrue(m_robot.shootCommand(ShotType.TEST));

        farmController.button(1).onTrue(new InstantCommand(shooter::increaseFlyFudge));
        farmController.button(2).onTrue(new InstantCommand(shooter::decreaseFlyFudge));

        // set the intake sensor position assuming it is deployed
        farmController.button(24).onTrue(new InstantCommand(() -> intake.getPivot().setPositionToDeployed()));

        farmController.button(6).onTrue(new InstantCommand(shooter::increaseFeedFudge));
        farmController.button(7).onTrue(new InstantCommand(shooter::decreaseFeedFudge));

        farmController.button(9).onTrue(new InstantCommand(shooter::increaseHoodFudge));
        farmController.button(10).onTrue(new InstantCommand(shooter::decreaseHoodFudge));

        farmController.button(4).onTrue(new InstantCommand(m_robot.getTurret()::increaseTurretFudge));
        farmController.button(5).onTrue(new InstantCommand(m_robot.getTurret()::decreaseTurretFudge));

        farmController.button(12).onTrue(new InstantCommand(intake.getRoller()::increaseIntakeFudge));
        farmController.button(14).onTrue(new InstantCommand(intake.getRoller()::decreaseIntakeFudge));

        farmController.button(3).onTrue(new InstantCommand(() -> shooter.setPassNeutral(true)));
        farmController.button(8).onTrue(new InstantCommand(() -> shooter.setPassNeutral(false)));

        // Reset the field-centric heading on Start press.
        driverController.view().onTrue(m_robot.getDrivetrain().runOnce(m_robot.getDrivetrain()::seedFieldCentric));
    }

    protected static void setDeadband(XboxController controller, double deadband) {
        controller.setLeftXDeadband(deadband);
        controller.setLeftYDeadband(deadband);

        controller.setRightXDeadband(deadband);
        controller.setRightYDeadband(deadband);

        controller.setLeftTriggerDeadband(deadband);
        controller.setRightTriggerDeadband(deadband);
    }

    private double conditionAxis(double value, SlewRateLimiter limiter) {
        // deadband is handled in the controller
        // Square the axis, retaining the sign
        double squared = Math.abs(value) * value;
        if (limiter == null)
            return squared;
        return limiter.calculate(squared);
    }

    private Command unjamCommand() {
        return new ParallelCommandGroup(
                new StartEndCommand(m_robot.getHopper()::reverse, m_robot.getHopper()::stop, m_robot.getHopper()),
                new StartEndCommand(m_robot.getShooterFeeder()::runReverseUnjam, m_robot.getShooterFeeder()::stop,
                        m_robot.getShooterFeeder()),
                m_robot.getIntake().outtakeCommand());
    }

    private Command unjamHopperCommand() {
        return new ParallelCommandGroup(
                new StartEndCommand(m_robot.getHopper()::reverse, m_robot.getHopper()::stop, m_robot.getHopper()),
                m_robot.getIntake().outtakeCommand());
    }
}
