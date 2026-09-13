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

    private final Telemetry m_swerveLogger = new Telemetry(MAX_SPEED, m_field2d);

    private final CommandNiDsXboxController m_driverController = new CommandNiDsXboxController(0);
    private final CommandJoystick m_farm = new CommandJoystick(1);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagVision m_aprilTagVision;
    private final ShooterFeeder m_shooterFeeder = new ShooterFeeder();
    private final Shooter m_shooter = new Shooter();
    private final Turret m_turret = new Turret(m_field2d);
    private final Intake m_intake = new Intake();
    private final Hopper m_hopper;

    
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

        m_aprilTagVision = new AprilTagVision(m_robotType, m_field2d);

        if (m_robotType == RobotType.TESTBOT) {
            m_drivetrain = new CommandSwerveDrivetrain(
                    m_aprilTagVision,
                    TunerConstantsTestBot.DrivetrainConstants,
                    TunerConstantsTestBot.FrontLeft, TunerConstantsTestBot.FrontRight, TunerConstantsTestBot.BackLeft,
                    TunerConstantsTestBot.BackRight);
        } else {
            m_drivetrain = new CommandSwerveDrivetrain(
                    m_aprilTagVision,
                    TunerConstantsCompBot.DrivetrainConstants,
                    TunerConstantsCompBot.FrontLeft, TunerConstantsCompBot.FrontRight, TunerConstantsCompBot.BackLeft,
                    TunerConstantsCompBot.BackRight);
        }

        m_hopper = new Hopper(m_drivetrain::getRobotCentricVelocity); 

        m_drivetrain.setupPathPlanner();
        m_drivetrain.registerTelemetry(m_swerveLogger::telemeterize);
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

    public Telemetry getSwerveLogger() {
        return m_swerveLogger;
    }

    public CommandNiDsXboxController getDriverController() {
        return m_driverController;
    }

    public CommandJoystick getFarmController() {
        return m_farm;
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return m_drivetrain;
    }

    public AprilTagVision getAprilTagVision() {
        return m_aprilTagVision;
    }

    public ShooterFeeder getShooterFeeder() {
        return m_shooterFeeder;
    }

    public Shooter getShooter() {
        return m_shooter;
    }

    public Turret getTurret() {
        return m_turret;
    }

    public Intake getIntake() {
        return m_intake;
    }

    public Hopper getHopper() {
        return m_hopper;
    }

    public DataLogger getDataLogger() {
        return m_dataLogger;
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
        return new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, m_drivetrain::getFieldCentricVelocity, shotType)
                .alongWith(new PulseHopper(m_hopper, m_shooter, m_turret));

        // new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)));
    }
    public Command shootCommand(double shotDistanceInches, Rotation2d turretHeading) {
        return new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, m_drivetrain::getFieldCentricVelocity, shotDistanceInches, turretHeading)
                .alongWith(new PulseHopper(m_hopper, m_shooter, m_turret));

        // new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)));
    }

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
