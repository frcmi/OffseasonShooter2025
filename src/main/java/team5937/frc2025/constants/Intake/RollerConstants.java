// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team5937.frc2025.constants.Intake;

import static edu.wpi.first.units.Units.*;
import static team5937.frc2025.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.Supplier;
import team5937.lib.subsystem.angular.AngularIOSimConfig;
import team5937.lib.subsystem.angular.AngularIOTalonFXConfig;
import team5937.lib.subsystem.angular.AngularSubsystemConfig;

public class RollerConstants {
    public static final Supplier<Rotation2d> kRealAngleFromSubsystemAngleZeroSupplier =
            () -> Rotation2d.kZero;

    public static final AngularSubsystemConfig kSubsystemConfigReal =
            AngularSubsystemConfig.builder()
                    .logKey("Pivot")
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

    public static final AngularIOSimConfig kSimConfig =
            AngularIOSimConfig.builder()
                    .motor(DCMotor.getKrakenX60Foc(3))
                    .build();
}

