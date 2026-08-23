package first.robot.opmode;

import org.wpilib.opmode.Teleop;

import first.robot.Robot;

@Teleop
public class OutreachTeleop extends NonZachTeleop {
    public OutreachTeleop(Robot robot) {
        super(robot, 0.5);
    }
}
