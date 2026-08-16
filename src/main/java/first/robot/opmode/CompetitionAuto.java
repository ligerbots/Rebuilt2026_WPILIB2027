// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.opmode;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.InternalButton;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import com.pathplanner.lib.events.EventTrigger;

import first.robot.FieldConstants;
import first.robot.Robot;
import first.robot.commands.autoCommands.AutoCommandInterface;
import first.robot.commands.autoCommands.CoreAuto;
import first.robot.subsystems.CommandSwerveDrivetrain;
import first.robot.subsystems.shooter.Shooter.ShotType;
import first.robot.utilities.AutoVisualizer;

@Autonomous(name = "My Auto")
public class CompetitionAuto extends PeriodicOpMode {        
    private AutoCommandInterface m_autoCommand;
    private boolean m_prevIsRedAlliance = true;
    private AutoVisualizer m_autoVisualizer = null;

    private final InternalButton m_virtualShootButton = new InternalButton();

    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>();
    private final SendableChooser<String> m_chosenAutoPaths = new SendableChooser<>();
    private final Map<String, List<Object>> m_autoPathOptions = new LinkedHashMap<>();
    private int m_autoSelectionCode = Integer.MIN_VALUE; 

    private final Robot m_robot;

    /** The Robot instance is passed into the opmode via the constructor. */
    public CompetitionAuto(Robot robot) {
        m_robot = robot;

        configureAutos();
    }
    
    @Override
    public void disabledPeriodic() {
        boolean isRedAlliance = FieldConstants.isRedAlliance();
        AutoCommandInterface newAuto = getAutonomousCommand();

        // don't change the initialPose unless the Auto or Alliance has changed
        // don't want to override the true pose on the field (as determined by the AprilTags)
        //
        // Note: use "==" to compare autos - checks if they are the same object
        if (isRedAlliance != m_prevIsRedAlliance || newAuto != m_autoCommand) {
            m_autoCommand = newAuto;
            m_prevIsRedAlliance = isRedAlliance;

            // drivetrain might be null when testing code. So check
            CommandSwerveDrivetrain driveTrain = m_robot.drivetrain;
            if (driveTrain != null) driveTrain.setPose(m_autoCommand.getInitialPose());
        }

        updateAutoPreview();
    }

    @Override
    public void start() {
        // clear the Auto preview 
        clearAutoPreview();

        // schedule the Auto command
        if (m_autoCommand != null)
            CommandScheduler.getInstance().schedule(m_autoCommand);
    }

    /**
     * This function is called asynchronously when the robot disables or switches
     * opmodes while this
     * opmode is enabled. Implementations should stop blocking work promptly.
     */
    @Override
    public  void end() {
        // not sure this is actually needed?
        if (m_autoCommand != null) {
            CommandScheduler.getInstance().cancel(m_autoCommand);
        }        
    }

    private void configureAutos() {

        // assign the Shoot button that is used during Autos
        // used only when shooting directly in the command
        // not used by PathPlanner triggers
        m_virtualShootButton.whileTrue(m_robot.shootCommand(ShotType.AUTO));

        addAutoOption("Depot Double Swipe Blitz", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ), true);
        
        addAutoOption("Depot Double Swipe Swing", List.of(
                "First Swipe Swing",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ));

        addAutoOption("BStart Depot Bump Only", List.of(
                "Bump Preload Bump",
                "BStart First Swipe Bump",
                "Bump Bump Shoot",
                "BStart First Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("BStart Depot Double Bump", List.of(
                "Bump Preload Trench",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Third Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Depot Double Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Depot Triple Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Depot Shoot"
                ));

        addAutoOption("Triple Swipe Bump", List.of(
                "First Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot",
                "Second Swipe Bump",
                "Bump Trench Shoot"
                ));

        addAutoOption("Triple Swipe Blitz", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

        addAutoOption("Triple Swipe Swing", List.of(
                "First Swipe Swing",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

        addAutoOption("Triple Swipe Pass", List.of(
                "First Swipe Blitz",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe Pass"
                ));

        addAutoOption("Pass Blitz", List.of(
                "Pass Swipe",
                "Pass Shoot"
                ));
        
        addAutoOption("Center Depot Simple Auto", List.of(
                "Hub Depot Shoot"
                ));

        addAutoOption("Swing Depot Double Swipe Blitz", List.of(
                "Swing First Swipe Blitz",
                "Swipe Shoot",
                "Depot Double Swipe Blitz"
                ));

        addAutoOption("Steal DOUBLE Swipe", List.of(
                "First Swipe Steal",
                "Swipe Shoot",
                "Depot Double Swipe Blitz",
                "Depot Trench Run Out"
                ));

        addAutoOption("Steal TRIPLE Swipe", List.of(
                "First Swipe Steal",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

         addAutoOption("4946 Triple Swipe Blitz", List.of(
                "4946 First Swipe Blitz",
                "Swipe Shoot",
                "Second Swipe",
                "Swipe Shoot Alt",
                "Third Swipe",
                "Swipe Shoot Alt"
                ));

        SmartDashboard.putData("Auto Choice", m_chosenAutoPaths);

        m_chosenFieldSide.setDefaultOption("Depot Side", "Depot Side");
        m_chosenFieldSide.addOption("Outpost Side", "Outpost Side");
        SmartDashboard.putData("Field Side", m_chosenFieldSide);

        SmartDashboard.putBoolean("autoStatus/runningIntake", false);
        SmartDashboard.putBoolean("autoStatus/runningShooter", false);

        configureAutoEventTriggers();
    }

    private void configureAutoEventTriggers() {
        new EventTrigger("Run Intake").onTrue(m_robot.intake.deployAndRollCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", true))));
        new EventTrigger("Stop Intake").onTrue(m_robot.intake.stowCommand().alongWith(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningIntake", false))));

        new EventTrigger("Shooter Running").whileTrue(m_robot.shootCommand(ShotType.AUTO));
        new EventTrigger("Shooter Running").onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("autoStatus/runningShooter", false)));

     }

    public AutoCommandInterface getAutonomousCommand() {
        String selectedAutoName = m_chosenAutoPaths.getSelected();
        String selectedFieldSide = m_chosenFieldSide.getSelected();
        int currentAutoSelectionCode = Objects.hash(
            selectedAutoName,
            selectedFieldSide,
            MatchState.getAlliance());

        // Only call constructor if the auto selection inputs have changed
        if (m_autoSelectionCode != currentAutoSelectionCode) {
            // double startT = Timer.getMonotonicTimestamp();

            m_autoSelectionCode = currentAutoSelectionCode;

            List<Object> selectedAutoPaths = m_autoPathOptions.get(selectedAutoName);
            boolean isOutpostSide = selectedFieldSide.equals("Outpost Side");

            m_autoVisualizer = new AutoVisualizer(m_robot.drivetrain.getPPRobotConfig());
            m_autoCommand = CoreAuto.getInstance(selectedAutoPaths, m_robot.drivetrain, isOutpostSide, m_virtualShootButton, m_autoVisualizer);

            SmartDashboard.putString("Selected Auto", selectedAutoName);
            m_autoVisualizer.registerAndStart(m_robot.getField2d());
            // System.out.println("*** Build Auto command took " + (Timer.getMonotonicTimestamp() - startT) + " seconds");
        }
        
        return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        return ((AutoCommandInterface) getAutonomousCommand()).getInitialPose();
    }    

    public void updateAutoPreview() {
        if (m_autoVisualizer != null) {
            m_autoVisualizer.update();
        }
    }
    
    public void clearAutoPreview() {
        if (m_autoVisualizer != null) {
            m_autoVisualizer.clear();
        }
    }
    
    private void addAutoOption(String name, List<Object> pathSteps) {
        addAutoOption(name, pathSteps, false);
    }

    private void addAutoOption(String name, List<Object> pathSteps, boolean isDefault) {
        m_autoPathOptions.put(name, pathSteps);
        if (isDefault) {
            m_chosenAutoPaths.setDefaultOption(name, name);
        } else {
            m_chosenAutoPaths.addOption(name, name);
        }
    }
}
