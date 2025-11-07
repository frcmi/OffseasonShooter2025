package team5937.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

@Builder
@Getter
public class AngularIOSimConfig {
    private final DCMotor motor;
    @Builder.Default private final MomentOfInertia moi = KilogramSquareMeters.of(1.0);
    @Builder.Default private final Angle resetAngle = Radians.of(0.0);
    @Builder.Default private final Angle physicalMinAngle = Radians.of(Double.NEGATIVE_INFINITY);
    @Builder.Default private final Angle physicalMaxAngle = Radians.of(Double.POSITIVE_INFINITY);
    @Builder.Default private final double motorRotationsPerOutputRotations = 1.0;
    @Builder.Default @Setter private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    @Builder.Default @Setter private double kP = 0.0;
    @Builder.Default @Setter private double kI = 0.0;
    @Builder.Default @Setter private double kD = 0.0;
    @Builder.Default @Setter private double kV = 0.0;
    @Builder.Default @Setter private AngularVelocity cruiseVelocity = RotationsPerSecond.of(0.0);

    @Builder.Default @Setter
    private AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0.0);
}
