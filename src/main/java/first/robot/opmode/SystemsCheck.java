package first.robot.opmode;

import org.wpilib.opmode.Utility;

import first.robot.Robot;

@Utility
public class SystemsCheck extends CompetitionTeleop {

    public SystemsCheck(Robot robot) {
        super(robot);

        // TODO: force a robot position
    }

    // for SystemsCheck no driving commands!
    @Override
    void driveBindings() {
    }
}
