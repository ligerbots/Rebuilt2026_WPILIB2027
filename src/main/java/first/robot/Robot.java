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
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.DataLogManager;

import com.ctre.phoenix6.HootAutoReplay;

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
    private double MAX_SPEED = SPEED_LIMIT * TunerConstantsCompBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final Telemetry m_swerveLogger = new Telemetry(MAX_SPEED);

    private final CommandNiDsXboxController m_driverController = new CommandNiDsXboxController(0);
    private final CommandJoystick m_farm = new CommandJoystick(1);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagVision m_aprilTagVision;
    private final ShooterFeeder m_shooterFeeder = new ShooterFeeder();
    private final Shooter m_shooter = new Shooter();
    private final Turret m_turret = new Turret(m_swerveLogger.getField2d());
    private final Intake m_intake = new Intake();
    private final Hopper m_hopper;

    // not used directly, but the periodic() method logs data
    @SuppressWarnings("unused")
    private final DataLogger m_dataLogger = new DataLogger();

    public static final String TESTBOT_SERIAL_NUMBER = "0313baff"; // TODO: real value?
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

        m_aprilTagVision = new AprilTagVision(m_robotType, m_swerveLogger.getField2d());

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
    public static RobotType getRobotType() {
        return m_robotType;
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
