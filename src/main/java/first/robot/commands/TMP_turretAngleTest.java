// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.Telemetry;

import first.robot.FieldConstants;
import first.robot.subsystems.shooter.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TMP_turretAngleTest extends Command {
  /** Creates a new TMP_turretAngleTest. */
  private Supplier<Pose2d> m_robotPose;
  private Turret m_Turret;

  public TMP_turretAngleTest(Supplier<Pose2d> robotPose, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotPose = robotPose;
    m_Turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d rotationGoal = Turret.getTranslationToGoal(m_robotPose.get(),
        FieldConstants.flipTranslation(FieldConstants.HUB_POSITION_BLUE)).getAngle().get();
    Telemetry.log("TurretAngleTest", true);
    Telemetry.log("turretTesting/ComputedAngle", rotationGoal.getDegrees());
    m_Turret.setAngle(rotationGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Telemetry.log("TurretAngleTest", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
