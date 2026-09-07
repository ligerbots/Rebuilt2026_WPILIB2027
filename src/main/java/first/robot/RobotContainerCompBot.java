// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.StartEndCommand;
import org.wpilib.command2.button.CommandJoystick;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.InternalButton;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.events.EventTrigger;

import first.robot.commands.PulseHopper;
import first.robot.commands.Shoot;
import first.robot.commands.autoCommands.AutoCommandInterface;
import first.robot.commands.autoCommands.CoreAuto;
import first.robot.generated.TunerConstantsCompBot;
import first.robot.subsystems.AprilTagVision;
import first.robot.subsystems.CommandSwerveDrivetrain;
import first.robot.subsystems.DataLogger;
import first.robot.subsystems.Hopper;
import first.robot.subsystems.intake.Intake;
import first.robot.subsystems.shooter.Shooter;
import first.robot.subsystems.shooter.Shooter.ShotType;
import first.robot.utilities.AutoVisualizer;
import first.robot.subsystems.shooter.ShooterFeeder;
import first.robot.subsystems.shooter.Turret;

public class RobotContainerCompBot extends RobotContainer {

    private static final double SPEED_LIMIT = 1.0;
    private double MAX_SPEED = SPEED_LIMIT * TunerConstantsCompBot.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private static final double JOYSTICK_DEADBAND = 0.05;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // set the swerve wheels in an X pattern
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveTelemetry m_logger = new SwerveTelemetry(MAX_SPEED);

    private AutoCommandInterface m_autoCommand;
    private AutoVisualizer m_autoVisualizer = null;

    private final CommandNiDsXboxController m_driverController = new CommandNiDsXboxController(0);
    private final CommandJoystick m_farm = new CommandJoystick(1);

    private final CommandSwerveDrivetrain m_drivetrain;
    private final AprilTagVision m_aprilTagVision = new AprilTagVision(Robot.RobotType.COMPBOT, m_logger.getField2d());
    private final ShooterFeeder m_shooterFeeder = new ShooterFeeder();
    private final Shooter m_shooter = new Shooter();
    private final Turret m_turret = new Turret(m_logger.getField2d());

    private final Intake m_intake = new Intake();
    private final Hopper m_hopper;

    private final InternalButton m_virtualShootButton = new InternalButton();

    // not used directly, but the periodic() method logs data
    @SuppressWarnings("unused")
    private final DataLogger m_dataLogger = new DataLogger();

    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>();
    private final SendableChooser<String> m_chosenAutoPaths = new SendableChooser<>();
    private final Map<String, List<Object>> m_autoPathOptions = new LinkedHashMap<>();
    private int m_autoSelectionCode = Integer.MIN_VALUE; 

    // Joystick slew rate limiters
    // higher value means faster change and more jumpy robot
    // max rate of 3 means maximum change = "3/sec"
    // so stick value will change 0 -> 1 in 0.33 seconds
    // remember that joysticks are -1 --> 1
    private static double JOYSTICK_MAX_SLEW_RATE = 3.0;
    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(JOYSTICK_MAX_SLEW_RATE);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(JOYSTICK_MAX_SLEW_RATE);
    private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(JOYSTICK_MAX_SLEW_RATE);
    
    public RobotContainerCompBot() {
        if (Robot.isSimulation()) {
            DriverStationBackend.silenceJoystickConnectionWarning(true);
        }
        
        m_drivetrain = new CommandSwerveDrivetrain(
            m_aprilTagVision,
            TunerConstantsCompBot.DrivetrainConstants,
            TunerConstantsCompBot.FrontLeft, TunerConstantsCompBot.FrontRight, TunerConstantsCompBot.BackLeft, TunerConstantsCompBot.BackRight
        );
        m_hopper = new Hopper(m_drivetrain::getRobotCentricVelocity); 

        m_drivetrain.setupPathPlanner();

        configureBindings();

        configureAutos();
    }

    private void configureAutos() {

        // assign the Shoot button that is used during Autos
        // used only when shooting directly in the command
        // not used by PathPlanner triggers
        m_virtualShootButton.whileTrue(getShootCommand());

        addAutoOption("Depot Double Swipe Blitz", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ), true);
        
        addAutoOption("Depot Double Swipe Swing", List.of(
                "First Swipe Swing",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ));

        addAutoOption("BStart Depot Bump Only", List.of(
                "Bump Preload Bump",
                "BStart First Swipe Bump",
                "Bump Bump Shoot",
                "BStart First Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("BStart Depot Double Bump", List.of(
                "Bump Preload Trench",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Third Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Depot Double Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Depot Triple Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Triple Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot"
                ));

        addAutoOption("Triple Swipe Blitz", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

        addAutoOption("Triple Swipe Swing", List.of(
                "First Swipe Swing",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

        addAutoOption("Triple Swipe Pass", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe Pass"
                ));

        addAutoOption("Pass Blitz", List.of(
                "Pass Swipe",
                "Pass Shoot"
                ));
        
        addAutoOption("Center Depot Simple Auto", List.of(
                "Hub Depot Shoot"
                ));

        addAutoOption("Swing Depot Double Swipe Blitz", List.of(
                "Swing First Swipe Blitz",
                "Swipe Shoot",
                "Depot Double Swipe Blitz"
                ));

        addAutoOption("Steal DOUBLE Swipe", List.of(
                "First Swipe Steal",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ));

        addAutoOption("Steal TRIPLE Swipe", List.of(
                "First Swipe Steal",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

         addAutoOption("4946 Triple Swipe Blitz", List.of(
                "4946 First Swipe Blitz",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

        SmartDashboard.putData("Auto Choice", m_chosenAutoPaths);

        m_chosenFieldSide.setDefaultOption("Depot Side", "Depot Side");
        m_chosenFieldSide.addOption("Outpost Side", "Outpost Side");
        SmartDashboard.putData("Field Side", m_chosenFieldSide);

        SmartDashboard.putBoolean("autoStatus/runningIntake", false);
        SmartDashboard.putBoolean("autoStatus/runningShooter", false);

        configureAutoEventTriggers();
    }

    public Command getShootCommand() {
        return withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder, m_drivetrain::getPose, m_drivetrain::getFieldCentricVelocity, ShotType.AUTO));
                        //     new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)));
    }
    
    private void configureAutoEventTriggers() {
        new EventTrigger("Run Intake").onTrue(m_intake.deployAndRollCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", true))));
        new EventTrigger("Stop Intake").onTrue(m_intake.stowCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", false))));

        new EventTrigger("Shooter Running").whileTrue(getShootCommand());
        new EventTrigger("Shooter Running").onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", false)));

     }

    private void configureBindings() {
        m_drivetrain.setDefaultCommand(getDriveCommand());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // enable/disable brake mode on the pivot when the robot is disabled
        RobotModeTriggers.disabled().onFalse(new InstantCommand(() -> m_intake.getPivot().setBrakeMode(false)));
        RobotModeTriggers.disabled().onTrue(
            new InstantCommand(() -> m_intake.getPivot().setBrakeMode(true)).ignoringDisable(true)
        );

        // Just shoot
        m_driverController.rightTrigger().whileTrue(getShootCommand());

        // shoot while intaking
        m_driverController.rightBumper().whileTrue(getShootCommand());
        m_driverController.rightBumper().onTrue(m_intake.getPivot().deployCommand());
        m_driverController.rightBumper().whileTrue(
                new StartEndCommand(m_intake.getRoller()::intake, m_intake.getRoller()::stop, m_intake.getRoller()));
                             
        // Deploy and run the intake (intake will stay out)
        m_driverController.leftTrigger().onTrue(m_intake.getPivot().deployCommand());
        m_driverController.leftTrigger().whileTrue(
                new StartEndCommand(m_intake.getRoller()::fastIntake, m_intake.getRoller()::stop, m_intake.getRoller()));
                        // .alongWith(new StartEndCommand(m_hopper::intake, m_hopper::stop, m_hopper)));

        // Stow the intake
        m_driverController.leftBumper().onTrue(m_intake.stowCommand());

        // lock wheels
        m_driverController.back().whileTrue(m_drivetrain.applyRequest(() -> m_brakeRequest));

        // Unjam
        m_farm.button(21).whileTrue(UnJamCommand());
        m_driverController.a().whileTrue(UnJamHopperCommand());
        m_driverController.b().whileTrue(UnJamHopperCommand());
        m_driverController.y().whileTrue(UnJamHopperCommand());
        m_driverController.x().whileTrue(UnJamHopperCommand());

        // fixed shots - distance in inches, plus ROBOT angle of turret
        // ladder - robot against the outside of the ladder, intake to the left for the dirver
        m_farm.button(11).whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder, 
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricVelocity, 130.0, Rotation2d.kCCW_90deg)));

        // corner shot
        m_farm.button(13).whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder,
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricVelocity, 210.0, Rotation2d.k180deg)));
        m_farm.button(15).whileTrue(withHopperControl(
                new Shoot(m_shooter, m_turret, m_shooterFeeder,
                        m_drivetrain::getPose, m_drivetrain::getFieldCentricVelocity, ShotType.TEST)));

        m_farm.button(1).onTrue(new InstantCommand(m_shooter::increaseFlyFudge));
        m_farm.button(2).onTrue(new InstantCommand(m_shooter::decreaseFlyFudge));

        // set the intake sensor position assuming it is deployed
        m_farm.button(24).onTrue(new InstantCommand(() -> m_intake.getPivot().setPositionToDeployed()));

        m_farm.button(6).onTrue(new InstantCommand(m_shooter::increaseFeedFudge));
        m_farm.button(7).onTrue(new InstantCommand(m_shooter::decreaseFeedFudge));

        m_farm.button(9).onTrue(new InstantCommand(m_shooter::increaseHoodFudge));
        m_farm.button(10).onTrue(new InstantCommand(m_shooter::decreaseHoodFudge));

        m_farm.button(4).onTrue(new InstantCommand(m_turret::increaseTurretFudge));
        m_farm.button(5).onTrue(new InstantCommand(m_turret::decreaseTurretFudge));

        m_farm.button(12).onTrue(new InstantCommand(m_intake.getRoller()::increaseIntakeFudge));
        m_farm.button(14).onTrue(new InstantCommand(m_intake.getRoller()::decreaseIntakeFudge));

        m_farm.button(3).onTrue(new InstantCommand(() -> m_shooter.setPassNeutral(true)));
        m_farm.button(8).onTrue(new InstantCommand(() -> m_shooter.setPassNeutral(false)));

        // Reset the field-centric heading on Start press.
        m_driverController.start().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driverController.back().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        // m_driverController.back().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driverController.start().and(m_driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driverController.start().and(m_driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        m_drivetrain.registerTelemetry(m_logger::telemeterize);


        // *** Test Commands *** 

        // m_driverController.y().whileTrue(new StartEndCommand(()->m_shooter.getFlywheel().setRPM(3000.0), ()->m_shooter.getFlywheel().stop()));

        // m_driverController.x().onTrue(new InstantCommand(() -> m_shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)))));
        
        // SmartDashboard.putNumber("flywheel/testVoltage", 0.0); 
        // m_farm.button(22).onTrue(new InstantCommand(() -> m_shooter.getFlywheel().setVoltage(SmartDashboard.getNumber("flywheel/testVoltage", 0.0))));

        // m_farm.button(23).onTrue(new InstantCommand(() -> m_shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0))));

        // SmartDashboard.putNumber("feeder/testVoltage", 0.0); 
        // m_farm.button(22).onTrue(new InstantCommand(() -> m_shooterFeeder.setKickerVoltage(SmartDashboard.getNumber("feeder/testVoltage", 0.0))));

        // m_farm.button(23).onTrue(new InstantCommand(() -> m_shooterFeeder.setKickerRPM(SmartDashboard.getNumber("kicker/testRPM", 0.0))));

        // m_driverController.a().onTrue(new InstantCommand(() -> m_shooterFeeder.setRPM(SmartDashboard.getNumber("shooterFeeder/testRPM", 0.0))));

        // SmartDashboard.putNumber("turret/testAngle", 0.0);
        // m_farm.button(22).onTrue(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("turret/testAngle", 0.0)))));

        // m_farm.button(23).whileTrue(
        //     new InstantCommand(() -> m_turret.setAngle(m_turret.getAngle().plus(Rotation2d.fromDegrees(4))))
        //         .andThen(new WaitCommand(0.018))
        //         .repeatedly()
        // );

        // m_farm.button(23).whileTrue(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(320.0)))
        //         .andThen(new WaitCommand(0.4))
        //         .andThen(new InstantCommand(() -> m_turret.setAngle(Rotation2d.fromDegrees(280.0))))
        // );

        // Command turretAngleTest = new TMP_turretAngleTest(m_drivetrain::getPose, m_turret);
        // m_driverController.start().whileTrue(turretAngleTest);
        // SmartDashboard.putBoolean("TurretAngleTest", false);
        // Trigger turretAngleTestTrigger = new Trigger(() -> SmartDashboard.getBoolean("TurretAngleTest", false));
        // turretAngleTestTrigger.whileTrue(turretAngleTest);
    }

    public CommandSwerveDrivetrain getDriveTrain() {
        return m_drivetrain;
    }

    public Command getAutonomousCommand() {
        String selectedAutoName = m_chosenAutoPaths.getSelected();
        String selectedFieldSide = m_chosenFieldSide.getSelected();
        int currentAutoSelectionCode = Objects.hash(
            selectedAutoName,
            selectedFieldSide,
            MatchState.getAlliance());

        // Only call constructor if the auto selection inputs have changed
        if (m_autoSelectionCode != currentAutoSelectionCode) {
            // double startT = Timer.getMonotonicTimestamp();

            m_autoSelectionCode = currentAutoSelectionCode;

            List<Object> selectedAutoPaths = m_autoPathOptions.get(selectedAutoName);
            boolean isOutpostSide = selectedFieldSide.equals("Outpost Side");

            m_autoVisualizer = new AutoVisualizer(m_drivetrain.getPPRobotConfig());
            m_autoCommand = CoreAuto.getInstance(selectedAutoPaths, m_drivetrain, isOutpostSide, m_virtualShootButton, m_autoVisualizer);

            SmartDashboard.putString("Selected Auto", selectedAutoName);
            m_autoVisualizer.registerAndStart(m_logger.getField2d());
            // System.out.println("*** Build Auto command took " + (Timer.getMonotonicTimestamp() - startT) + " seconds");
        }
        
        return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        return ((AutoCommandInterface) getAutonomousCommand()).getInitialPose();
    }    

    public void updateAutoPreview() {
        if (m_autoVisualizer != null) {
            m_autoVisualizer.update();
        }
    }
    
    public void clearAutoPreview() {
        if (m_autoVisualizer != null) {
            m_autoVisualizer.clear();
        }
    }
    
    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation

        return m_drivetrain.applyRequest(() ->
                m_driveRequest.withVelocityX(-conditionAxis(m_driverController.getLeftY(), m_xLimiter) * MAX_SPEED)
                    .withVelocityY(-conditionAxis(m_driverController.getLeftX(), m_yLimiter) * MAX_SPEED)
                    .withRotationalRate(-conditionAxis(m_driverController.getRightX(), m_rotationLimiter) * MAX_ANGULAR_RATE)
                );
    }

    private double conditionAxis(double value, SlewRateLimiter limiter) {
        value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        double squared = Math.abs(value) * value;
        return limiter.calculate(squared);
    }

    private Command UnJamCommand() {
        return new ParallelCommandGroup(
                new StartEndCommand(m_hopper::reverse, m_hopper::stop, m_hopper),
                new StartEndCommand(m_shooterFeeder::runReverseUnjam, m_shooterFeeder::stop, m_shooterFeeder),
                m_intake.outtakeCommand());
    }

    private Command UnJamHopperCommand() {
        return new ParallelCommandGroup(
                new StartEndCommand(m_hopper::reverse, m_hopper::stop, m_hopper),
                m_intake.outtakeCommand());
    }

    private Command withHopperControl(Command shootCommand) {
        return shootCommand.alongWith(new PulseHopper(m_hopper, m_shooter, m_turret));
    }

    private void addAutoOption(String name, List<Object> pathSteps) {
        addAutoOption(name, pathSteps, false);
    }

    private void addAutoOption(String name, List<Object> pathSteps, boolean isDefault) {
        m_autoPathOptions.put(name, pathSteps);
        if (isDefault) {
            m_chosenAutoPaths.setDefaultOption(name, name);
        } else {
            m_chosenAutoPaths.addOption(name, name);
        }
    }
}
