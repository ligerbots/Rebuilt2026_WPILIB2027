// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.intake;

import static org.wpilib.units.Units.Seconds;
import org.wpilib.command3.Command;

import first.robot.subsystems.PeriodicMechanism;

public class Intake extends PeriodicMechanism {
    
    final IntakePivot m_intakePivot;
    final IntakeRoller m_intakeRoller;
    
    /** Creates a new Intake. */
    public Intake() {
        super();
        
        m_intakePivot = new IntakePivot();
        m_intakeRoller = new IntakeRoller();
    }

    public IntakeRoller getRoller() {
        return m_intakeRoller;
    }

    public IntakePivot getPivot() {
        return m_intakePivot;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    public Command stowCommand() {
        Command waitCmd = Command.requiring(m_intakePivot, m_intakeRoller)
                .executing(
                        coroutine -> {
                            coroutine.waitUntil(m_intakePivot::onTarget);
                        })
                .named("foo").withTimeout(Seconds.of(1));

        return Command.sequence(
            m_intakeRoller.runRollers(),
            m_intakePivot.setAngleCommand(IntakePivot.STOW_POSITION),
            waitCmd,
            m_intakePivot.holdAngleCommand(IntakePivot.STOW_POSITION),
            m_intakeRoller.stopRollers()).withAutomaticName();
    }
    
    public Command deployAndRollCommand() {
        return m_intakePivot.deployCommand().alongWith(runFastRollers()).withAutomaticName();
    }
    
    public Command deployCommand() {
        return m_intakePivot.deployCommand();
    }
    
    public Command runRollers() {
        return m_intakeRoller.runRollers();
    }
    
    public Command runFastRollers() {
        return m_intakeRoller.runFastRollers();
    }
    
    public Command stopRollers() {
        return m_intakeRoller.stopRollers();
    }

    public Command outtakeCommand() {
        return m_intakePivot.deployCommand().alongWith(m_intakeRoller.outtakeThenStop()).withAutomaticName();
    }
}
