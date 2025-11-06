package team5937.lib.sim;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import team5937.frc2025.constants.ModeConstants;
import team5937.lib.subsystem.VirtualSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CurrentDrawCalculatorSim extends VirtualSubsystem {
    private final List<Supplier<Current>> subsystemCurrentDraws = new ArrayList<>();

    @SafeVarargs
    public final void registerCurrentDraw(Supplier<Current>... voltageDraws) {
        Collections.addAll(subsystemCurrentDraws, voltageDraws);
    }

    public void periodic() {
        if (!ModeConstants.kCurrentMode.equals(ModeConstants.Mode.kSim)) return;

        double[] draws =
                subsystemCurrentDraws.stream()
                        .mapToDouble(current -> current.get().in(Amps))
                        .toArray();

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(draws));

        Logger.recordOutput(
                "CurrentDrawCalculatorSim/BatteryVoltage", RobotController.getBatteryVoltage());
    }
}
