package first.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command3.Mechanism;

public class PeriodicMechanism extends Mechanism {
    // This is not the best, but its a start. Just a hack for testing, really
    private static final List<PeriodicMechanism> s_mechanisms = new ArrayList<PeriodicMechanism>();

    public PeriodicMechanism() {
        s_mechanisms.add(this);
    }

    public void periodic() {}

    public void postCommand() {}

    static void runPeriodics() {
        for (PeriodicMechanism mech : s_mechanisms) 
            mech.periodic();
    }

    static void runPostCommands() {
        for (PeriodicMechanism mech : s_mechanisms) 
            mech.postCommand();
    }
}
