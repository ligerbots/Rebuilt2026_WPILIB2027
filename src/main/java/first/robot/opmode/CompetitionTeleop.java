// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.StartEndCommand;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import first.robot.Robot;
import first.robot.commands.PulseHopper;
import first.robot.commands.Shoot;
import first.robot.subsystems.shooter.Shooter.ShotType;

@Teleop
public class CompetitionTeleop extends PeriodicOpMode {
    private static final double JOYSTICK_DEADBAND = 0.05;

    // Setting up bindings for necessary control of the swerve drive platform
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // set the swerve wheels in an X pattern
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private final Robot m_robot;

    /** The Robot instance is passed into the opmode via the constructor. */
    public CompetitionTeleop(Robot robot) {
        m_robot = robot;

        // for convenience here, since they are used so frequently below
        CommandNiDsXboxController driverController = m_robot.driverController;
        CommandGenericHID farmController = m_robot.farm;

        robot.drivetrain.setDefaultCommand(driveCommand());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                robot.drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // enable/disable brake mode on the pivot when the robot is disabled
        RobotModeTriggers.disabled().onFalse(new InstantCommand(() -> robot.intake.getPivot().setBrakeMode(false)));
        RobotModeTriggers.disabled().onTrue(
                new InstantCommand(() -> robot.intake.getPivot().setBrakeMode(true)).ignoringDisable(true));

        // Just shoot
        driverController.rightTrigger().whileTrue(shootCommand());

        // shoot while intaking
        driverController.rightBumper().whileTrue(shootCommand());
        driverController.rightBumper().onTrue(robot.intake.getPivot().deployCommand());
        driverController.rightBumper().whileTrue(
                new StartEndCommand(robot.intake.getRoller()::intake, robot.intake.getRoller()::stop, robot.intake.getRoller()));

        // Deploy and run the intake (intake will stay out)
        driverController.leftTrigger().onTrue(robot.intake.getPivot().deployCommand());
        driverController.leftTrigger().whileTrue(
                new StartEndCommand(robot.intake.getRoller()::fastIntake, robot.intake.getRoller()::stop,
                        robot.intake.getRoller()));
        // .alongWith(new StartEndCommand(m_robot.hopper::intake, m_robot.hopper::stop, m_robot.hopper)));

        // Stow the intake
        driverController.leftBumper().onTrue(robot.intake.stowCommand());

        // lock wheels
        driverController.back().whileTrue(robot.drivetrain.applyRequest(() -> m_brakeRequest));

        // Unjam
        farmController.button(21).whileTrue(unjamCommand());
        driverController.a().whileTrue(unjamHopperCommand());
        driverController.b().whileTrue(unjamHopperCommand());
        driverController.y().whileTrue(unjamHopperCommand());
        driverController.x().whileTrue(unjamHopperCommand());

        // fixed shots - distance in inches, plus ROBOT angle of turret
        // ladder - robot against the outside of the ladder, intake to the left for the
        // dirver
        farmController.button(11).whileTrue(withHopperControl(
                new Shoot(robot.shooter, robot.turret, robot.shooterFeeder,
                        robot.drivetrain::getPose, robot.drivetrain::getFieldCentricVelocity, 130.0, Rotation2d.kCCW_90deg)));

        // corner shot
        farmController.button(13).whileTrue(withHopperControl(
                new Shoot(robot.shooter, robot.turret, robot.shooterFeeder,
                        robot.drivetrain::getPose, robot.drivetrain::getFieldCentricVelocity, 210.0, Rotation2d.k180deg)));
        farmController.button(15).whileTrue(withHopperControl(
                new Shoot(robot.shooter, robot.turret, robot.shooterFeeder,
                        robot.drivetrain::getPose, robot.drivetrain::getFieldCentricVelocity, ShotType.TEST)));

        farmController.button(1).onTrue(new InstantCommand(robot.shooter::increaseFlyFudge));
        farmController.button(2).onTrue(new InstantCommand(robot.shooter::decreaseFlyFudge));

        // set the intake sensor position assuming it is deployed
        farmController.button(24).onTrue(new InstantCommand(() -> robot.intake.getPivot().setPositionToDeployed()));

        farmController.button(6).onTrue(new InstantCommand(robot.shooter::increaseFeedFudge));
        farmController.button(7).onTrue(new InstantCommand(robot.shooter::decreaseFeedFudge));

        farmController.button(9).onTrue(new InstantCommand(robot.shooter::increaseHoodFudge));
        farmController.button(10).onTrue(new InstantCommand(robot.shooter::decreaseHoodFudge));

        farmController.button(4).onTrue(new InstantCommand(robot.turret::increaseTurretFudge));
        farmController.button(5).onTrue(new InstantCommand(robot.turret::decreaseTurretFudge));

        farmController.button(12).onTrue(new InstantCommand(robot.intake.getRoller()::increaseIntakeFudge));
        farmController.button(14).onTrue(new InstantCommand(robot.intake.getRoller()::decreaseIntakeFudge));

        farmController.button(3).onTrue(new InstantCommand(() -> robot.shooter.setPassNeutral(true)));
        farmController.button(8).onTrue(new InstantCommand(() -> robot.shooter.setPassNeutral(false)));

        // Reset the field-centric heading on Start press.
        driverController.start().onTrue(robot.drivetrain.runOnce(robot.drivetrain::seedFieldCentric));
    }

    @Override
    public void disabledPeriodic() {
        /* Called periodically (on every DS packet) while the robot is disabled. */
    }

    @Override
    public void start() {
        /* Called once when the robot is enabled. */
    }

    @Override
    public void periodic() {
        /* Called periodically (set time interval) while the robot is enabled. */
    }

    @Override
    public void end() {
        /* Called when the robot is disabled (after previously being enabled). */
    }

    @Override
    public void close() {
        /*
         * Called when the opmode is de-selected / no additional methods will be called.
         */
    }

    public Command driveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation

        // NOTE: for competition with our best driver, no slew limiters
        return m_robot.drivetrain.applyRequest(() ->
                m_driveRequest.withVelocityX(-conditionAxis(m_robot.driverController.getLeftY()) * Robot.MAX_SPEED)
                    .withVelocityY(-conditionAxis(m_robot.driverController.getLeftX()) * Robot.MAX_SPEED)
                    .withRotationalRate(-conditionAxis(m_robot.driverController.getRightX()) * Robot.MAX_ANGULAR_RATE)
                );
    }

    private double conditionAxis(double value) {
        value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        return Math.abs(value) * value;
    }

    public Command shootCommand() {
        return withHopperControl(
                new Shoot(m_robot.shooter, m_robot.turret, m_robot.shooterFeeder, m_robot.drivetrain::getPose, m_robot.drivetrain::getFieldCentricVelocity, ShotType.AUTO));
                        //     new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)));
    }

    private Command unjamCommand() {
        return new ParallelCommandGroup(
                new StartEndCommand(m_robot.hopper::reverse, m_robot.hopper::stop, m_robot.hopper),
                new StartEndCommand(m_robot.shooterFeeder::runReverseUnjam, m_robot.shooterFeeder::stop, m_robot.shooterFeeder),
                m_robot.intake.outtakeCommand());
    }

    private Command unjamHopperCommand() {
        return new ParallelCommandGroup(
                new StartEndCommand(m_robot.hopper::reverse, m_robot.hopper::stop, m_robot.hopper),
                m_robot.intake.outtakeCommand());
    }

    private Command withHopperControl(Command shootCommand) {
        return shootCommand.alongWith(new PulseHopper(m_robot.hopper, m_robot.shooter, m_robot.turret));
    }
}
