// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5937.frc2025.constants.Intake;

import static edu.wpi.first.units.Units.*;
import static team5937.frc2025.constants.RobotConstants.kRioBus;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;

import java.util.function.Supplier;
import team5937.lib.subsystem.angular.AngularIOSimConfig;
import team5937.lib.subsystem.angular.AngularIOTalonFXConfig;
import team5937.lib.subsystem.angular.AngularSubsystemConfig;

public class RollerConstants {
    public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
            () -> Rotation2d.kZero;

    public static final AngularSubsystemConfig kSubsystemConfigReal =
            AngularSubsystemConfig.builder()
                .logKey("Roller")
                .bus(kRioBus)
                .build();

    public static final AngularIOTalonFXConfig kTalonFXConfig =
            AngularIOTalonFXConfig.builder()
                    .masterId(32)
                    .followerId(31)
                    .followerId(30)
                    .bus(kRioBus)
                    .build();

    public static final AngularSubsystemConfig kSubsystemConfigSim =
            AngularSubsystemConfig.builder()
                    .logKey(kSubsystemConfigReal.getLogKey())
                    .bus(kSubsystemConfigReal.getBus())
                    .build();
    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.242366 * 3 * 0.000292639653); // Converted from lb in^2 to kg m^2, multiply by 3 for 3 rollers
    public static final AngularIOSimConfig kSimConfig =
            AngularIOSimConfig.builder()
                    .motor(DCMotor.getKrakenX60(1))
                    .moi(kMOI)
                    .build();
}

