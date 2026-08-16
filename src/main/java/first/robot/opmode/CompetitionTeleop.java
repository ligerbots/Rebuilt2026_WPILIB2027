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
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import first.robot.Robot;
import first.robot.subsystems.shooter.Shooter.ShotType;

@Teleop
public class CompetitionTeleop extends PeriodicOpMode {
    protected static final double JOYSTICK_DEADBAND = 0.05;

    // Setting up bindings for necessary control of the swerve drive platform
    protected final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // set the swerve wheels in an X pattern
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();

    // This is "protected" so that the subclasses can access it
    protected final Robot m_robot;

    /** The Robot instance is passed into the opmode via the constructor. */
    public CompetitionTeleop(Robot robot) {
        m_robot = robot;

        driveBindings();
        mainBindings();
    }

    // The binding configuration is broken into a couple of routines so that
    // different pieces
    void driveBindings() {
        m_robot.drivetrain.setDefaultCommand(
                m_robot.drivetrain.applyRequest(() -> m_driveRequest
                        .withVelocityX(-conditionAxis(m_robot.driverController.getLeftY()) * Robot.MAX_SPEED)
                        .withVelocityY(-conditionAxis(m_robot.driverController.getLeftX()) * Robot.MAX_SPEED)
                        .withRotationalRate(-conditionAxis(m_robot.driverController.getRightX()) * Robot.MAX_ANGULAR_RATE)));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                m_robot.drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    }

    void mainBindings() {
        // for convenience here, since they are used so frequently below
        CommandNiDsXboxController driverController = m_robot.driverController;
        CommandGenericHID farmController = m_robot.farm;

        // enable/disable brake mode on the pivot when the robot is disabled
        RobotModeTriggers.disabled().onFalse(new InstantCommand(() -> m_robot.intake.getPivot().setBrakeMode(false)));
        RobotModeTriggers.disabled().onTrue(
                new InstantCommand(() -> m_robot.intake.getPivot().setBrakeMode(true)).ignoringDisable(true));

        // Just shoot
        driverController.rightTrigger().whileTrue(m_robot.shootCommand(ShotType.AUTO));

        // shoot while intaking
        driverController.rightBumper().whileTrue(m_robot.shootCommand(ShotType.AUTO));
        driverController.rightBumper().onTrue(m_robot.intake.getPivot().deployCommand());
        driverController.rightBumper().whileTrue(
                new StartEndCommand(m_robot.intake.getRoller()::intake, m_robot.intake.getRoller()::stop, m_robot.intake.getRoller()));

        // Deploy and run the intake (intake will stay out)
        driverController.leftTrigger().onTrue(m_robot.intake.getPivot().deployCommand());
        driverController.leftTrigger().whileTrue(
                new StartEndCommand(m_robot.intake.getRoller()::fastIntake, m_robot.intake.getRoller()::stop,
                        m_robot.intake.getRoller()));
        // .alongWith(new StartEndCommand(m_robot.hopper::intake, m_robot.hopper::stop, m_m_robot.hopper)));

        // Stow the intake
        driverController.leftBumper().onTrue(m_robot.intake.stowCommand());

        // lock wheels
        driverController.back().whileTrue(m_robot.drivetrain.applyRequest(() -> m_brakeRequest));

        // Unjam
        farmController.button(21).whileTrue(unjamCommand());
        driverController.a().whileTrue(unjamHopperCommand());
        driverController.b().whileTrue(unjamHopperCommand());
        driverController.y().whileTrue(unjamHopperCommand());
        driverController.x().whileTrue(unjamHopperCommand());

        // fixed shots - distance in inches, plus ROBOT angle of turret
        // ladder - robot against the outside of the ladder, intake to the left for the
        // dirver
        farmController.button(11).whileTrue(m_robot.shootCommand(130.0, Rotation2d.kCCW_90deg));

        // corner shot
        farmController.button(13).whileTrue(m_robot.shootCommand(210.0, Rotation2d.k180deg));
        // TODO: move all test code to a new OpMode
        farmController.button(15).whileTrue(m_robot.shootCommand(ShotType.TEST));

        farmController.button(1).onTrue(new InstantCommand(m_robot.shooter::increaseFlyFudge));
        farmController.button(2).onTrue(new InstantCommand(m_robot.shooter::decreaseFlyFudge));

        // set the intake sensor position assuming it is deployed
        farmController.button(24).onTrue(new InstantCommand(() -> m_robot.intake.getPivot().setPositionToDeployed()));

        farmController.button(6).onTrue(new InstantCommand(m_robot.shooter::increaseFeedFudge));
        farmController.button(7).onTrue(new InstantCommand(m_robot.shooter::decreaseFeedFudge));

        farmController.button(9).onTrue(new InstantCommand(m_robot.shooter::increaseHoodFudge));
        farmController.button(10).onTrue(new InstantCommand(m_robot.shooter::decreaseHoodFudge));

        farmController.button(4).onTrue(new InstantCommand(m_robot.turret::increaseTurretFudge));
        farmController.button(5).onTrue(new InstantCommand(m_robot.turret::decreaseTurretFudge));

        farmController.button(12).onTrue(new InstantCommand(m_robot.intake.getRoller()::increaseIntakeFudge));
        farmController.button(14).onTrue(new InstantCommand(m_robot.intake.getRoller()::decreaseIntakeFudge));

        farmController.button(3).onTrue(new InstantCommand(() -> m_robot.shooter.setPassNeutral(true)));
        farmController.button(8).onTrue(new InstantCommand(() -> m_robot.shooter.setPassNeutral(false)));

        // Reset the field-centric heading on Start press.
        driverController.start().onTrue(m_robot.drivetrain.runOnce(m_robot.drivetrain::seedFieldCentric));
    }

    // @Override
    // public void disabledPeriodic() {
    //     /* Called periodically (on every DS packet) while the robot is disabled. */
    // }

    // @Override
    // public void start() {
    //     /* Called once when the robot is enabled. */
    // }

    // @Override
    // public void periodic() {
    //     /* Called periodically (set time interval) while the robot is enabled. */
    // }

    // @Override
    // public void end() {
    //     /* Called when the robot is disabled (after previously being enabled). */
    // }

    // @Override
    // public void close() {
    //     /*
    //      * Called when the opmode is de-selected / no additional methods will be called.
    //      */
    // }

    private double conditionAxis(double value) {
        value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        return Math.abs(value) * value;
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
}
