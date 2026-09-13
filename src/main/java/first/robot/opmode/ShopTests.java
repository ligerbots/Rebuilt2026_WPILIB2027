package first.robot.opmode;

import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.StartEndCommand;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.smartdashboard.SmartDashboard;

import first.robot.Robot;
import first.robot.commands.TMP_turretAngleTest;
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
        CommandGenericHID farmController = m_robot.getFarmController();
        Shooter shooter = m_robot.getShooter();

        farmController.button(15).whileTrue(m_robot.shootCommand(ShotType.TEST));


        // *** Test Commands *** 

        driverController.y().whileTrue(new StartEndCommand(()->shooter.getFlywheel().setRPM(3000.0), ()->shooter.getFlywheel().stop()));

        driverController.x().onTrue(new InstantCommand(() -> shooter.getHood().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("hood/testAngle", 0.0)))));
        
        SmartDashboard.putNumber("flywheel/testVoltage", 0.0); 
        farmController.button(22).onTrue(new InstantCommand(() -> shooter.getFlywheel().setVoltage(SmartDashboard.getNumber("flywheel/testVoltage", 0.0))));

        farmController.button(23).onTrue(new InstantCommand(() -> shooter.getFlywheel().setRPM(SmartDashboard.getNumber("flywheel/testRPM", 0.0))));

        SmartDashboard.putNumber("feeder/testVoltage", 0.0); 
        farmController.button(22).onTrue(new InstantCommand(() -> m_robot.getShooterFeeder().setKickerVoltage(SmartDashboard.getNumber("feeder/testVoltage", 0.0))));

        farmController.button(23).onTrue(new InstantCommand(() -> m_robot.getShooterFeeder().setKickerRPM(SmartDashboard.getNumber("kicker/testRPM", 0.0))));

        driverController.a().onTrue(new InstantCommand(() -> m_robot.getShooterFeeder().setFeederBeltsRPM(SmartDashboard.getNumber("m_robot.getShooterFeeder()/testRPM", 0.0))));

        SmartDashboard.putNumber("turret/testAngle", 0.0);
        farmController.button(22).onTrue(new InstantCommand(() -> m_robot.getTurret().setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("turret/testAngle", 0.0)))));

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
        driverController.start().whileTrue(turretAngleTest);
        SmartDashboard.putBoolean("TurretAngleTest", false);
        Trigger turretAngleTestTrigger = new Trigger(() -> SmartDashboard.getBoolean("TurretAngleTest", false));
        turretAngleTestTrigger.whileTrue(turretAngleTest);
    }
}
