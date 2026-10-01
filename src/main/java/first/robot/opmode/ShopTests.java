package first.robot.opmode;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.StartEndCommand;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import first.robot.Robot;
import first.robot.commands.TMP_turretAngleTest;
import first.robot.subsystems.shooter.Shooter;
import first.robot.subsystems.shooter.Shooter.ShotType;

@Utility
public class ShopTests extends PeriodicOpMode {
    private Robot m_robot;

    private final TunableDouble m_flywheelVoltage = Tunables.addDouble("flywheel/testVoltage", 0.0);
    private final TunableDouble m_flywheelRpm = Tunables.addDouble("flywheel/testRPM", 0.0);
    private final TunableDouble m_hoodAngleDeg = Tunables.addDouble("hood/testAngle", 0.0);
    private final TunableDouble m_feederVoltage = Tunables.addDouble("feeder/testVoltage", 0.0);
    private final TunableDouble m_feederRpm = Tunables.addDouble("feeder/testRPM", 0.0);
    private final TunableDouble m_kickerRpm = Tunables.addDouble("kicker/testRPM", 0.0);
    private final TunableDouble m_turrentAngleDeg = Tunables.addDouble("turret/testAngle", 0.0);
    private final TunableBoolean m_runTurretTest = Tunables.addBoolean("runTurretTest", false);

    public ShopTests(Robot robot)
    {
        m_robot = robot;

        configureBindings();
    }

    private void configureBindings() {
        // for convenience here, since they are used so frequently below
        CommandXboxController driverController = m_robot.getDriverController();
        CommandGenericHID farmController = m_robot.getFarmController();
        Shooter shooter = m_robot.getShooter();

        farmController.button(15).whileTrue(m_robot.shootCommand(ShotType.TEST));


        // *** Test Commands *** 

        driverController.y().whileTrue(new StartEndCommand(()->shooter.getFlywheel().setRPM(3000.0), ()->shooter.getFlywheel().stop()));

        driverController.x().onTrue(new InstantCommand(() -> shooter.getHood().setAngle(Rotation2d.fromDegrees(m_hoodAngleDeg.get()))));
        
        farmController.button(22).onTrue(new InstantCommand(() -> shooter.getFlywheel().setVoltage(m_flywheelVoltage.get())));

        farmController.button(23).onTrue(new InstantCommand(() -> shooter.getFlywheel().setRPM(m_flywheelRpm.get())));

        Telemetry.log("feeder/testVoltage", 0.0); 
        farmController.button(22).onTrue(new InstantCommand(() -> m_robot.getShooterFeeder().setKickerVoltage(m_feederVoltage.get())));

        farmController.button(23).onTrue(new InstantCommand(() -> m_robot.getShooterFeeder().setKickerRPM(m_kickerRpm.get())));

        driverController.a().onTrue(new InstantCommand(() -> m_robot.getShooterFeeder().setFeederBeltsRPM(m_feederRpm.get())));

        Telemetry.log("turret/testAngle", 0.0);
        farmController.button(22).onTrue(new InstantCommand(() -> m_robot.getTurret().setAngle(Rotation2d.fromDegrees(m_turrentAngleDeg.get()))));

        farmController.button(23).whileTrue(
            new InstantCommand(() -> m_robot.getTurret().setAngle(m_robot.getTurret().getAngle().plus(Rotation2d.fromDegrees(4))))
                .andThen(new WaitCommand(0.018))
                .repeatedly()
        );

        farmController.button(23).whileTrue(new InstantCommand(() -> m_robot.getTurret().setAngle(Rotation2d.fromDegrees(320.0)))
                .andThen(new WaitCommand(0.4))
                .andThen(new InstantCommand(() -> m_robot.getTurret().setAngle(Rotation2d.fromDegrees(280.0))))
        );

        Command turretAngleTest = new TMP_turretAngleTest(m_robot.getDrivetrain()::getPose, m_robot.getTurret());
        driverController.menu().whileTrue(turretAngleTest);
        Trigger turretAngleTestTrigger = new Trigger(() -> m_runTurretTest.get());
        turretAngleTestTrigger.whileTrue(turretAngleTest);
    }
}
