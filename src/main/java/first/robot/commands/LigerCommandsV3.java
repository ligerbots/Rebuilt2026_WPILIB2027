package first.robot.commands;

import org.wpilib.command3.Command;

public class LigerCommandsV3 {
    public static Command InstantCommand(String name, Runnable runnable) {
        return Command.noRequirements(
                coroutine -> {
                    runnable.run();
                }).named(name);
    }

    public static Command StartEndCommand(String name, Runnable start, Runnable end) {
        return Command.noRequirements(
                coroutine -> {
                    start.run();
                    coroutine.park(); // wait until canceled
                }).whenCanceled(end).named(name);
    }
}
