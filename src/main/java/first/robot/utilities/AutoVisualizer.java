package first.robot.utilities;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.system.Timer;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import first.robot.FieldConstants;

public class AutoVisualizer {
    private static final String PREVIEW_PATH_NAME = "selectedAutoPath";
    private static final String PREVIEW_ROBOT_NAME = "selectedAutoActor";
    
    private Field2d m_field = null;
    private RobotConfig m_robotConfig;

    private List<Pose2d> m_previewPoses = new ArrayList<>();
    private List<PathPlannerTrajectory> m_previewTrajectories = new ArrayList<>();
    private double m_previewDurationSec = 0.0;
    private Timer m_previewTimer = new Timer();

    public AutoVisualizer(RobotConfig config) {
        m_robotConfig = config;
    }

    public void registerAndStart(Field2d field) {
        m_field = field;
        m_previewTimer.restart();

        m_field.getObject(PREVIEW_PATH_NAME).setPoses(m_previewPoses);
        update();
    }

    public void update() {
        Pose2d previewPoseBlue = getAnimatedPreviewPoseBlue();
        if (previewPoseBlue == null) {
            m_field.getObject(PREVIEW_ROBOT_NAME).setPoses();
        } else {
            m_field.getObject(PREVIEW_ROBOT_NAME).setPose(FieldConstants.flipPose(previewPoseBlue));
        }
    }

    public void clear() {
        m_field.getObject(PREVIEW_PATH_NAME).setPoses();
        m_field.getObject(PREVIEW_ROBOT_NAME).setPoses();
    }

    public void addPath(PathPlannerPath path) {
        for (Pose2d pose : path.getPathPoses()) {
            m_previewPoses.add(FieldConstants.flipPose(pose));
        }

        if (m_robotConfig == null) return;

        PathPlannerTrajectory trajectory = buildAutoPreviewTrajectory(path);
        if (trajectory == null) return;

        m_previewTrajectories.add(trajectory);
        m_previewDurationSec += trajectory.getTotalTimeSeconds();
    }

    // ----------------------------------------------------------------

    private PathPlannerTrajectory buildAutoPreviewTrajectory(PathPlannerPath path) {
        Rotation2d startingRotation = getPreviewStartingRotation(path);
        double startingSpeedMps = path.getIdealStartingState() != null ? path.getIdealStartingState().velocityMPS() : 0.0;
        Rotation2d pathHeading = getPathHeading(path);
        Translation2d fieldVelocity = new Translation2d(startingSpeedMps, pathHeading);
        ChassisVelocities startingSpeeds = new ChassisVelocities(fieldVelocity.getX(), fieldVelocity.getY(), 0.0).toRobotRelative(startingRotation);

        return path.generateTrajectory(startingSpeeds, startingRotation, m_robotConfig);
    }

    private Rotation2d getPreviewStartingRotation(PathPlannerPath path) {
        if (path.getIdealStartingState() != null) {
            return path.getIdealStartingState().rotation();
        }

        return getPathHeading(path);
    }

    private Rotation2d getPathHeading(PathPlannerPath path) {
        List<Pose2d> pathPoses = path.getPathPoses();
        if (pathPoses.size() < 2) {
            return Rotation2d.ZERO;
        }

        Translation2d headingVector = pathPoses.get(1).getTranslation().minus(pathPoses.get(0).getTranslation());
        if (headingVector.getNorm() < 1e-6) {
            return Rotation2d.ZERO;
        }

        return headingVector.getAngle();
    }

    private Pose2d getAnimatedPreviewPoseBlue() {
        if (m_previewTrajectories.isEmpty()) {
            if (m_previewPoses.isEmpty()) {
                return null;
            }

            return m_previewPoses.get(0);
        }

        if (m_previewDurationSec > 0.0) {
            double elapsedSec = m_previewTimer.get();
            double previewTimeSec = elapsedSec % m_previewDurationSec;

            for (PathPlannerTrajectory trajectory : m_previewTrajectories) {
                double trajectoryDurationSec = trajectory.getTotalTimeSeconds();
                if (previewTimeSec <= trajectoryDurationSec) {
                    return trajectory.sample(previewTimeSec).pose;
                }

                previewTimeSec -= trajectoryDurationSec;
            }
        }

        // return the end pose
        return m_previewTrajectories.get(m_previewTrajectories.size() - 1).getEndState().pose;
    }
}
