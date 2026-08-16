// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.*;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.CommandJoystick;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.hal.HALUtil;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.DataLogManager;

import com.ctre.phoenix6.HootAutoReplay;

import first.robot.commands.PulseHopper;
import first.robot.commands.Shoot;
import first.robot.generated.TunerConstantsCompBot;
import first.robot.generated.TunerConstantsTestBot;
import first.robot.subsystems.AprilTagVision;
import first.robot.subsystems.CommandSwerveDrivetrain;
import first.robot.subsystems.DataLogger;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.intake.Intake;
import first.robot.subsystems.shooter.Shooter;
import first.robot.subsystems.shooter.ShooterFeeder;
import first.robot.subsystems.shooter.Turret;
import first.robot.utilities.HubShiftUtil;

public class Robot extends OpModeRobot {
    private static final double SPEED_LIMIT = 1.0;
    public static final double MAX_SPEED = SPEED_LIMIT * TunerConstantsCompBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Field2d m_field2d = new Field2d();

    public final Telemetry m_swerveLogger = new Telemetry(MAX_SPEED, m_field2d);

    public final CommandNiDsXboxController driverController = new CommandNiDsXboxController(0);
    public final CommandJoystick farm = new CommandJoystick(1);

    public final CommandSwerveDrivetrain drivetrain;
    public final AprilTagVision aprilTagVision;
    public final ShooterFeeder shooterFeeder = new ShooterFeeder();
    public final Shooter shooter = new Shooter();
    public final Turret turret = new Turret(m_field2d);
    public final Intake intake = new Intake();
    public final Hopper hopper;

    
    // not used directly, but the periodic() method logs data
    @SuppressWarnings("unused")
    private final DataLogger m_dataLogger = new DataLogger();

    public static final String TESTBOT_SERIAL_NUMBER = "0313baff";
    public static final String COMPBOT_SERIAL_NUMBER = "030fc268";

    public enum RobotType {
        TESTBOT, COMPBOT
    }

    // we want this to be static so that it is easy for subsystems to query the
    // robot type
    private static RobotType m_robotType;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        if (Robot.isSimulation()) {
            DriverStationBackend.silenceJoystickConnectionWarning(true);
        }
        
        // Enable local logging.
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        determineRobotType();

        // Create the subsystems (aka Mechanisms)
        // If there are differences between robots, create the correct version here

        aprilTagVision = new AprilTagVision(m_robotType, m_field2d);

        if (m_robotType == RobotType.TESTBOT) {
            drivetrain = new CommandSwerveDrivetrain(
                    aprilTagVision,
                    TunerConstantsTestBot.DrivetrainConstants,
                    TunerConstantsTestBot.FrontLeft, TunerConstantsTestBot.FrontRight, TunerConstantsTestBot.BackLeft,
                    TunerConstantsTestBot.BackRight);
        } else {
            drivetrain = new CommandSwerveDrivetrain(
                    aprilTagVision,
                    TunerConstantsCompBot.DrivetrainConstants,
                    TunerConstantsCompBot.FrontLeft, TunerConstantsCompBot.FrontRight, TunerConstantsCompBot.BackLeft,
                    TunerConstantsCompBot.BackRight);
        }

        hopper = new Hopper(drivetrain::getRobotCentricVelocity); 

        drivetrain.setupPathPlanner();
        drivetrain.registerTelemetry(m_swerveLogger::telemeterize);
    }

    private void determineRobotType() {
        // Figure out which roboRio this is, so we know which version of the robot
        //   code to run.
        String serialNum = HALUtil.getSerialNumber();
        SmartDashboard.putString("rioSerialNumber", serialNum);
        if (serialNum.equals(TESTBOT_SERIAL_NUMBER)) {
            m_robotType = RobotType.TESTBOT;
        } else if (serialNum.equals(COMPBOT_SERIAL_NUMBER)) {
            m_robotType = RobotType.COMPBOT;
        } else {
            // default to the Test robot unless we're running in simulation
            m_robotType = isSimulation() ? RobotType.COMPBOT : RobotType.TESTBOT;
        }
        SmartDashboard.putString("robotType", m_robotType.toString());
    }

    // Useful if a subsystem needs to know which chassis
    public RobotType getRobotType() {
        return m_robotType;
    }

    public Field2d getField2d() {
        return m_field2d;
    }

    // @Override
    // public void driverStationConnected() {
    // }

    // /**
    //  * This function is called periodically anytime when no opmode is selected,
    //  * including when the
    //  * Driver Station is disconnected.
    //  */
    // @Override
    // public void nonePeriodic() {
    // }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {
        HubShiftUtil.disable();
    }

    // Uses most of the Subsystems, and needed in Teleop and Auto
    public Command shootCommand(Shooter.ShotType shotType) {
        return new Shoot(shooter, turret, shooterFeeder, drivetrain::getPose, drivetrain::getFieldCentricVelocity, shotType)
                .alongWith(new PulseHopper(hopper, shooter, turret));

        // new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)));
    }
    public Command shootCommand(double shotDistanceInches, Rotation2d turretHeading) {
        return new Shoot(shooter, turret, shooterFeeder, drivetrain::getPose, drivetrain::getFieldCentricVelocity, shotDistanceInches, turretHeading)
                .alongWith(new PulseHopper(hopper, shooter, turret));

        // new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)));
    }

    // @Override
    // public void disabledPeriodic() {
    //     boolean isRedAlliance = FieldConstants.isRedAlliance();
    //     Command newAuto = m_robotContainer.getAutonomousCommand();

    //     // don't change the initialPose unless the Auto or Alliance has changed
    //     // don't want to override the true pose on the field (as determined by the AprilTags)
    //     //
    //     // Note: use "==" to compare autos - checks if they are the same object
    //     if (isRedAlliance != m_prevIsRedAlliance || newAuto != m_autonomousCommand) {
    //         m_autonomousCommand = newAuto;
    //         m_prevIsRedAlliance = isRedAlliance;

    //         // drivetrain might be null when testing code. So check
    //         CommandSwerveDrivetrain driveTrain = m_robotContainer.getDriveTrain();
    //         if (driveTrain != null) driveTrain.setPose(m_robotContainer.getInitialPose());
    //     }

    //     m_robotContainer.updateAutoPreview();
    // }

    // @Override
    // public void disabledExit() {
    //     m_robotContainer.clearAutoPreview();
    // }

    // @Override
    // public void autonomousInit() {
    //     // double startT = Timer.getMonotonicTimestamp();
    //     m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //     if (m_autonomousCommand != null) {
    //         CommandScheduler.getInstance().schedule(m_autonomousCommand);
    //     }
    //     // System.out.println("*** AutoInit took " + (Timer.getMonotonicTimestamp() - startT) + " seconds");
    // }

    // @Override
    // public void autonomousPeriodic() {}

    // @Override
    // public void autonomousExit() {}

    // @Override
    // public void teleopInit() {
    //     HubShiftUtil.initialize();
    //     // note: use this here, or in disabledExit(), but no need for both
    //     // m_robotContainer.clearAutoPreview();

    //     if (m_autonomousCommand != null) {
    //         CommandScheduler.getInstance().cancel(m_autonomousCommand);
    //     }
    // }

    // @Override
    // public void teleopPeriodic() {}

    // @Override
    // public void teleopExit() {}
}
