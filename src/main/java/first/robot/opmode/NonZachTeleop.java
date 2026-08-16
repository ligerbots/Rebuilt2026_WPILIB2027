package first.robot.opmode;

import org.wpilib.command2.Command;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.Teleop;

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

    public NonZachTeleop(Robot robot) {
        super(robot);
    }

    // Different drive command for our less experienced drivers
    // Include slewRateLimiters to smooth the control
    @Override
    public Command driveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation

        return m_robot.drivetrain.applyRequest(() ->
                m_driveRequest.withVelocityX(-conditionAxis(m_robot.driverController.getLeftY(), m_xLimiter) * Robot.MAX_SPEED)
                    .withVelocityY(-conditionAxis(m_robot.driverController.getLeftX(), m_yLimiter) * Robot.MAX_SPEED)
                    .withRotationalRate(-conditionAxis(m_robot.driverController.getRightX(), m_rotationLimiter) * Robot.MAX_ANGULAR_RATE)
                );
    }

    private double conditionAxis(double value, SlewRateLimiter limiter) {
        value = MathUtil.applyDeadband(value, CompetitionTeleop.JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        double squared = Math.abs(value) * value;
        return limiter.calculate(squared);
    }
}
