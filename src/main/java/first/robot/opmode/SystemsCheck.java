package first.robot.opmode;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.Utility;

import first.robot.FieldConstants;
import first.robot.Robot;

@Utility
public class SystemsCheck extends CompetitionTeleop {
    static final Pose2d INITIAL_POSE = new Pose2d(2.0, FieldConstants.FIELD_WIDTH/2.0, Rotation2d.kZero);

    public SystemsCheck(Robot robot) {
        super(robot);

        // set the robot at a known position so that shooting is predictable without tags
        m_robot.getDrivetrain().setPose(INITIAL_POSE);
    }

    // for SystemsCheck no driving commands!
    @Override
    void driveBindings() {
    }
}
