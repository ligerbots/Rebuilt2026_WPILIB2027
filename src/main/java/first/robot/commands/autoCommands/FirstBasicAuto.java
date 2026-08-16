
package first.robot.commands.autoCommands;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import first.robot.FieldConstants;
import first.robot.subsystems.CommandSwerveDrivetrain;

public class FirstBasicAuto extends AutoCommandInterface {

    protected Pose2d m_initPose;
    private CommandSwerveDrivetrain m_driveTrain;
    
    PathConstraints constraints = new PathConstraints(
            4.0, 2.0,
            Math.toRadians(540), Math.toRadians(720));

    /** Creates a new FirstBasicAuto. */
    public FirstBasicAuto(CommandSwerveDrivetrain driveTrain, boolean isDepotSide) {

        m_driveTrain = driveTrain;

        try {
            
            String[] pathNames = {
                    "Start Bump to Fuel Begin",
                    "Fuel Begin to Fuel End",
                    "Fuel End to Bump Finish",
                    "Bump Finish to Climb A"
            };

            PathPlannerPath startPath = PathPlannerPath.fromPathFile(pathNames[0]);
            if (isDepotSide) {
                startPath = startPath.mirrorPath();
            }
            m_initPose = startPath.getStartingHolonomicPose().get();

            for (String pathName : pathNames) {
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
                if(isDepotSide) {
                    path = path.mirrorPath();
                }
                addCommands(m_driveTrain.followPath(path));
            }
        } catch (Exception e) {
            DriverStationErrors.reportError("Unable to load PP path Test", true);
            m_initPose = new Pose2d();
        }
    }
    
    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }
}
