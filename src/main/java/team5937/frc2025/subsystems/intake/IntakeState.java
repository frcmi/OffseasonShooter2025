// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5937.frc2025.subsystems.intake;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import lombok.Setter;
import team5937.lib.LoggedTunableNumber;
import team5937.lib.utils.StructUtils;

import static edu.wpi.first.units.Units.*;

import java.nio.ByteBuffer;
import java.util.Optional;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class IntakeState implements StructSerializable{
    @Setter private Angle pivot;
    @Setter private Voltage rollers;
    private final String type;

    private final Optional<LoggedTunableNumber> pivotTunable;
    private final Optional<LoggedTunableNumber> rollerVelocityTunable;

    public IntakeState(Angle pivot, Voltage rollers) {
        this.pivot = pivot;
        this.rollers = rollers;

        pivotTunable = Optional.empty();
        rollerVelocityTunable = Optional.empty();

        type = "kNotTunable";
    }

    public IntakeState(Voltage rollers, Angle pivot, String logKey) {
        this.pivot = pivot;
        this.rollers = rollers;
        this.type = logKey;

        pivotTunable =
                Optional.of(
                        new LoggedTunableNumber(
                                String.format("IntakeStates/%s/PivotAngleDegrees", logKey),
                                pivot.in(Degrees)));
        rollerVelocityTunable =
                Optional.of(
                        new LoggedTunableNumber(
                                String.format("ArmStates/%s/RollersVolts", logKey),
                                rollers.in(Volts)));
    }

    public Angle getPivot() {
        return pivotTunable
            .map(loggedTunableNumber -> Degrees.of(loggedTunableNumber.get())).orElse(pivot);
    }

    public Voltage getRollers() {
        return rollerVelocityTunable
            .map(loggedTunableNumber -> Volts.of(loggedTunableNumber.get())).orElse(rollers);
    }

    @SuppressWarnings("unused")
    public static final Struct<IntakeState> struct =
            new Struct<>() {
                @Override
                public Class<IntakeState> getTypeClass() {
                    return IntakeState.class;
                }

                @Override
                public String getTypeName() {
                    return "IntakeState";
                }

                @Override
                public int getSize() {
                    return kSizeDouble * 2 + 256;
                }

                @Override
                public String getSchema() {
                    // spotless:off
                    return "double pivotAngleDegrees;double rollerVoltageVolts;char Type[256]";
                    // spotless:on
                }

                @Override
                public IntakeState unpack(ByteBuffer bb) {
                    Angle pivot = Degrees.of(bb.getDouble());
                    Voltage rollers = Volts.of(bb.getDouble());
                    String type = StructUtils.readString(bb, 256);

                    return new IntakeState(rollers, pivot, type);
                }

                @Override
                public void pack(ByteBuffer bb, IntakeState value) {
                    bb.putDouble(value.getPivot().in(Degrees));
                    bb.putDouble(value.getRollers().in(Volts));
                    StructUtils.writeString(bb, value.type, 256);
                }

                @Override
                public boolean isImmutable() {
                    return true;
                }
            };
}