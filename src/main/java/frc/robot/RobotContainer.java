// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public abstract class RobotContainer {
    public abstract Command getAutonomousCommand();
    public abstract Pose2d getInitialPose();

    public abstract CommandSwerveDrivetrain getDriveTrain();
    public abstract void clearAutoPreview();
    public abstract void updateAutoPreviewActor();
}
