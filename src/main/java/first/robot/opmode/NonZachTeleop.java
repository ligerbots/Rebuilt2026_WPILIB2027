package first.robot.opmode;

import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.Teleop;

import com.ctre.phoenix6.swerve.SwerveRequest;

import first.robot.Robot;

@Teleop
public class NonZachTeleop extends CompetitionTeleop {
    // Joystick slew rate limiters
    // higher value means faster change and more jumpy robot
    // max rate of 3 means maximum change = "3/sec"
    // so stick value will change 0 -> 1 in 0.33 seconds
    // remember that joysticks are -1 --> 1
    private static double JOYSTICK_MAX_SLEW_RATE = 3.0;
    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(JOYSTICK_MAX_SLEW_RATE);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(JOYSTICK_MAX_SLEW_RATE);
    private SlewRateLimiter m_rotationLimiter = new SlewRateLimiter(JOYSTICK_MAX_SLEW_RATE);

    private final double m_speedScale;

    public NonZachTeleop(Robot robot) {
        this(robot, 1.0);
    }

    public NonZachTeleop(Robot robot, double speedScale) {
        m_speedScale = speedScale;
        super(robot);
    }

    // Different drive command for our less experienced drivers
    // Include slewRateLimiters to smooth the control
    @Override
    void driveBindings() {
        m_robot.drivetrain.setDefaultCommand(
                m_robot.drivetrain.applyRequest(() ->
                m_driveRequest.withVelocityX(-conditionAxis(m_robot.driverController.getLeftY(), m_xLimiter) * Robot.MAX_SPEED * m_speedScale)
                    .withVelocityY(-conditionAxis(m_robot.driverController.getLeftX(), m_yLimiter) * Robot.MAX_SPEED * m_speedScale)
                    .withRotationalRate(-conditionAxis(m_robot.driverController.getRightX(), m_rotationLimiter) * Robot.MAX_ANGULAR_RATE * m_speedScale)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                m_robot.drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    }

    private double conditionAxis(double value, SlewRateLimiter limiter) {
        value = MathUtil.applyDeadband(value, CompetitionTeleop.JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        double squared = Math.abs(value) * value;
        return limiter.calculate(squared);
    }
}
