package team5937.lib.subsystem.sensor.currentsensor;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.*;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;
import team5937.lib.subsystem.angular.AngularSubsystem;
import team5937.lib.subsystem.linear.LinearSubsystem;

@Builder
@Getter
public class CurrentSensorSubsystemConfig {
    private final String logKey;
    private final BooleanSupplier connectedSupplier;
    private final Supplier<Current> currentSupplier;
    @Setter private Current threshold;
    @Builder.Default @Setter private Time debounce = Seconds.of(0.0);

    public static CurrentSensorSubsystemConfig fromLinearSubsystem(
            LinearSubsystem subsystem, Current threshold, Time debounce, String logKey) {
        return builder()
                .logKey(logKey)
                .connectedSupplier(subsystem::areAllDevicesConnected)
                .currentSupplier(subsystem::getSupplyCurrent)
                .threshold(threshold)
                .debounce(debounce)
                .build();
    }

    public static CurrentSensorSubsystemConfig fromAngularSubsystem(
            AngularSubsystem subsystem, Current threshold, Time debounce, String logKey) {
        return builder()
                .logKey(logKey)
                .connectedSupplier(subsystem::areAllDevicesConnected)
                .currentSupplier(subsystem::getSupplyCurrent)
                .threshold(threshold)
                .debounce(debounce)
                .build();
    }
}
