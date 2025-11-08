package team5937.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import team5937.frc2025.constants.Intake.PivotConstants;
import team5937.frc2025.subsystems.intake.IntakeState;
import team5937.lib.subsystem.VirtualSubsystem;

public class SuperstructureVisualizer extends VirtualSubsystem {
    private final double PADDING = 22.0; // inches

    private final LoggedMechanism2d mechanism =
    new LoggedMechanism2d(
            Inches.of(PADDING + 23 + PADDING).in(Meters), // Width: 23in
            Inches.of(95.0).in(Meters),
            new Color8Bit(34, 34, 34));
    private final LoggedMechanismRoot2d pivotRoot;
    private final LoggedMechanismLigament2d pivot;
    private final LoggedMechanismLigament2d pivotSupport;

    private final Supplier<IntakeState> intakeState;

    private final String logKey;

    public SuperstructureVisualizer(Supplier<IntakeState> intakeState, String logKey, Color8Bit mechColor) {
        this.intakeState = intakeState;
        this.logKey = logKey;

        pivotRoot = mechanism.getRoot("PivotRoot", 
            Inches.of(PADDING + 23 - 1.536513).in(Meters), // Relative to sideplate
            Inches.of(0.75 + 12.504).in(Meters)); // 0.75 is ground clearance, 12.504 is above origin (which has ground clearance)

        // Slapdown intake visualization
        pivot = pivotRoot.append(new LoggedMechanismLigament2d(
            "Pivot",
            PivotConstants.PivotLength.in(Meters),
            intakeState.get()
                .getPivot()
                .in(Degrees),
            8.0,
            mechColor));

        // The base of the slapdown intake
        pivotSupport = pivotRoot.append(new LoggedMechanismLigament2d(
            "PivotSupport",
            Inches.of(12.4).in(Meters),
            -90.0,
            8.0,
            new Color8Bit(0, 0, 0)));
    }

    @Override
    public void periodic() {
        pivot.setAngle(intakeState.get().getPivot().in(Degrees));
        Logger.recordOutput(String.format("Superstructure/%s", logKey), mechanism);
    }
}
