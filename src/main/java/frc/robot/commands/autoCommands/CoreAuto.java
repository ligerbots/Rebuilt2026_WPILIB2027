package frc.robot.commands.autoCommands;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.InternalButton;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CoreAuto extends AutoCommandInterface {

    protected Pose2d m_initPose;
    private CommandSwerveDrivetrain m_driveTrain;

    PathConstraints constraints = new PathConstraints(
            4.0, 2.0,
            Math.toRadians(540), Math.toRadians(720));

    public static CoreAuto getInstance(List<Object> pathSteps, CommandSwerveDrivetrain driveTrain, boolean isOutpostSide,
            InternalButton virtualShootButton) {
        return new CoreAuto(pathSteps, driveTrain, isOutpostSide, virtualShootButton);
    }
    
    /** Creates a new CoreAuto. 
     * @param m_shooter 
     * @param m_turret 
     */
    private CoreAuto(List<Object> pathSteps, CommandSwerveDrivetrain driveTrain, boolean isOutpostSide,
            InternalButton virtualShootButton) {

        m_driveTrain = driveTrain;

        SmartDashboard.putBoolean("autoStatus/runningIntake", false);
        SmartDashboard.putBoolean("autoStatus/runningShooter", false);

        try {
            m_initPose = getStartPose(pathSteps, isOutpostSide);

            for (Object step : pathSteps) {
                if (step instanceof Number) {
                    addCommands(shootForNSeconds(((Number) step).doubleValue(), virtualShootButton));
                } else if (step instanceof String) {
                    PathPlannerPath path = PathPlannerPath.fromPathFile((String) step);
                    if (isOutpostSide) {
                        path = path.mirrorPath();
                    }
                    addCommands(m_driveTrain.followPath(path));
                } else {
                    DriverStation.reportError("Invalid auto step: " + step.toString(), true);
                }
            }
            // Clmber code goes here, but we don't have a climber yet so we'll leave it out for now
        } catch (Exception e) {
            DriverStation.reportError("Unable to load PP path Test", true);
            m_initPose = new Pose2d();
        }
    }

    private Command shootForNSeconds(double seconds, InternalButton virtualShootButton) {   
        return new InstantCommand(() -> virtualShootButton.setPressed(true))
                .alongWith(new WaitCommand(seconds),
                        new InstantCommand(
                                () -> SmartDashboard.putBoolean("autoStatus/runningShooter", true)))
                .andThen(new InstantCommand(() -> virtualShootButton.setPressed(false)),
                        new InstantCommand(
                                () -> SmartDashboard.putBoolean("autoStatus/runningShooter", false)));
    }
    
    private Pose2d getStartPose(List<Object> pathSteps, boolean isOutpostSide) {
        try {
            PathPlannerPath startPath = null;
            // Assume start path is either the first or second element in the list
            if ((pathSteps.get(0) instanceof String)) {
                startPath = PathPlannerPath.fromPathFile((String) pathSteps.get(0));
            } else {
                startPath = PathPlannerPath.fromPathFile((String) pathSteps.get(1));
            }
            if (isOutpostSide) {
                startPath = startPath.mirrorPath();
            }
            return startPath.getStartingHolonomicPose().get();
        } catch (Exception e) {
            DriverStation.reportError("Unable to load PP path for initial pose", true);
            return new Pose2d();
        }
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }
}
