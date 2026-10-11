package first.robot.opmode;

import org.wpilib.opmode.Teleop;

import first.robot.Robot;

@Teleop
public class NonZachTeleop extends CompetitionTeleop {
    // Joystick slew rate limiters
    // higher value means faster change and more jumpy robot
    // max rate of 3 means maximum change = "3/sec"
    // so stick value will change 0 -> 1 in 0.33 seconds
    // remember that joysticks are -1 --> 1
    private static double JOYSTICK_DRIVE_MAX_SLEW_RATE = 3.0;
    private static double JOYSTICK_TURN_MAX_SLEW_RATE = 5.0;

    public NonZachTeleop(Robot robot) {
        this(robot, 1.0);
    }

    public NonZachTeleop(Robot robot, double speedScale) {
        super(robot, JOYSTICK_DRIVE_MAX_SLEW_RATE, JOYSTICK_TURN_MAX_SLEW_RATE, speedScale);
    }
}
