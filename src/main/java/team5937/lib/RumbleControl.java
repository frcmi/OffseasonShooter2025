package team5937.lib;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team5937.lib.command.CommandsUtil;
import team5937.lib.command.RepeatCommandWithEnd;

import java.util.function.BiConsumer;

public class RumbleControl {
    public final Runnable stopRumble;
    private final BiConsumer<RumbleType, Double> rumbleJoystick;

    public RumbleControl(BiConsumer<RumbleType, Double> rumbleJoystick) {
        this.rumbleJoystick = rumbleJoystick;
        this.stopRumble = () -> rumbleJoystick.accept(RumbleType.kBothRumble, 0.0);
    }

    public void addTrigger(Trigger trigger, RumbleType type, double intensity, double seconds) {
        trigger.onTrue(
                CommandsUtil.repeatTimes(
                        sequence(
                                runOnce(() -> rumbleJoystick.accept(type, intensity)),
                                waitSeconds(0.6),
                                runOnce(stopRumble)),
                        3));
    }

    public Command rumble(double intensity, double seconds) {
        return startEnd(() -> rumbleJoystick.accept(RumbleType.kBothRumble, intensity), stopRumble)
                .withTimeout(seconds);
    }

    public Command rumbleCommand(Trigger trigger) {
        return sequence(
                        waitSeconds(0.1),
                        waitUntil(trigger),
                        new RepeatCommandWithEnd(rumble(1, 0.1), 5))
                .finallyDo(stopRumble);
    }
}
