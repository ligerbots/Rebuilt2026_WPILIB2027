// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands.autoCommands;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command3.Command;

public abstract class AutoCommandInterface implements Command {

    public abstract Pose2d getInitialPose();
}