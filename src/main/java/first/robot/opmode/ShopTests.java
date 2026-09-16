package first.robot.opmode;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandJoystick;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.smartdashboard.SmartDashboard;

import first.robot.Robot;
import first.robot.commands.LigerCommandsV3;
// import first.robot.commands.TMP_turretAngleTest;
import first.robot.subsystems.shooter.Shooter;
import first.robot.subsystems.shooter.Shooter.ShotType;

@Utility
public class ShopTests extends PeriodicOpMode {
    private Robot m_robot;

    public ShopTests(Robot robot)
    {
        m_robot = robot;

        configureBindings();
    }

    private void configureBindings() {
        // for convenience here, since they are used so frequently below
        CommandNiDsXboxController driverController = m_robot.getDriverController();
        CommandJoystick farmController = m_robot.getFarmController();
        Shooter shooter = m_robot.getShooter();

        farmController.button(15).whileTrue(m_robot.shootCommand(ShotType.TEST));


        // *** Test Commands *** 

        driverController.y().whileTrue(LigerCommandsV3.StartEndCommand("run3000", ()->shooter.getFlywheel().setRPM(3000.0), ()->shooter.getFlywheel().stop()));

        driverController.x().onTrue(LigerCommandsV3.InstantCommand("testAngle", () -> shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)))));
        
        SmartDashboard.putNumber("flywheel/testVoltage", 0.0); 
        farmController.button(1).onTrue(LigerCommandsV3.InstantCommand("flyTestVoltage", () -> shooter.getFlywheel().setVoltage(SmartDashboard.getNumber("flywheel/testVoltage", 0.0))));

        farmController.button(2).onTrue(LigerCommandsV3.InstantCommand("flyTestRPM", () -> shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0))));

        SmartDashboard.putNumber("feeder/testVoltage", 0.0); 
        farmController.button(3).onTrue(LigerCommandsV3.InstantCommand("feederTestVoltage", () -> m_robot.getShooterFeeder().setKickerVoltage(SmartDashboard.getNumber("feeder/testVoltage", 0.0))));

        farmController.button(4).onTrue(LigerCommandsV3.InstantCommand("feederTestRPM", () -> m_robot.getShooterFeeder().setKickerRPM(SmartDashboard.getNumber("kicker/testRPM", 0.0))));

        driverController.a().onTrue(LigerCommandsV3.InstantCommand("beltsTestRPM", () -> m_robot.getShooterFeeder().setFeederBeltsRPM(SmartDashboard.getNumber("belts/testRPM", 0.0))));

        SmartDashboard.putNumber("turret/testAngle", 0.0);
        farmController.button(5).onTrue(LigerCommandsV3.InstantCommand("turrentTestAngle", () -> m_robot.getTurret().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("turret/testAngle", 0.0)))));

        farmController.button(6).whileTrue(
                Command.noRequirements(
                        coroutine -> {
                            while (true) {
                                m_robot.getTurret()
                                        .setAngle(m_robot.getTurret().getAngle().plus(Rotation2d.fromDegrees(4)));
                                coroutine.wait(Seconds.of(0.018));
                            }
                        }).named("stepTurret"));

        farmController.button(23).whileTrue(
                LigerCommandsV3
                        .InstantCommand("goto320", () -> m_robot.getTurret().setAngle(Rotation2d.fromDegrees(320.0)))
                        .andThen(Command.waitFor(Seconds.of(0.4)).named(""))
                        .andThen(LigerCommandsV3.InstantCommand("goto280",
                                () -> m_robot.getTurret().setAngle(Rotation2d.fromDegrees(280.0))))
                        .withAutomaticName());

        // Command turretAngleTest = LigerCommandsV3.TMP_turretAngleTest(m_robot.getDrivetrain()::getPose, m_robot.getTurret());
        // driverController.start().whileTrue(turretAngleTest);

        // SmartDashboard.putBoolean("TurretAngleTest", false);
        // Trigger turretAngleTestTrigger = LigerCommandsV3.Trigger(() -> SmartDashboard.getBoolean("TurretAngleTest", false));
        // turretAngleTestTrigger.whileTrue(turretAngleTest);
    }
}
