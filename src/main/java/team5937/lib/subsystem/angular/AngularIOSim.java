package team5937.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static team5937.frc2025.constants.RobotConstants.kDt;
import static team5937.lib.subsystem.angular.AngularIOOutputMode.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import team5937.lib.sim.CurrentDrawCalculatorSim;
import team5937.lib.sim.PivotSim;
import team5937.lib.subsystem.DeviceConnectedStatus;

import java.util.Optional;
import java.util.function.Supplier;

public class AngularIOSim implements AngularIO {
    private final PivotSim pivot;
    private final ProfiledPIDController controller;

    private final AngularIOSimConfig deviceConfig;

    private final double[] motorTemperatures = new double[] {};
    private final DeviceConnectedStatus[] deviceConnectedStatuses = new DeviceConnectedStatus[] {};

    private AngularIOOutputMode outputMode = kNeutral;
    private Optional<Angle> goal = Optional.empty();
    private Optional<Double> dutyCycle = Optional.empty();

    private Current statorCurrent = Amps.of(0.0);

    private Optional<Supplier<Rotation2d>> realAngleFromSubsystemAngleZero = Optional.empty();
    private Optional<Supplier<Distance>> armLength = Optional.empty();

    private AngularVelocity velocity = RadiansPerSecond.of(0.0);

    public AngularIOSim(
            AngularIOSimConfig config, CurrentDrawCalculatorSim currentDrawCalculatorSim) {
        this.deviceConfig = config;

        // Hardware
        DCMotor motor = config.getMotor();
        pivot =
                new PivotSim(
                        motor,
                        config.getMotorRotationsPerOutputRotations(),
                        config.getMoi().in(KilogramSquareMeters),
                        armLength.orElse(() -> Meters.of(0.0)).get().in(Meters),
                        config.getPhysicalMinAngle().in(Radians),
                        config.getPhysicalMaxAngle().in(Radians),
                        realAngleFromSubsystemAngleZero,
                        config.getResetAngle().in(Radians));
        controller =
                new ProfiledPIDController(
                        config.getKP(),
                        config.getKI(),
                        config.getKD(),
                        new TrapezoidProfile.Constraints(
                                config.getCruiseVelocity().in(RadiansPerSecond),
                                config.getAcceleration().in(RadiansPerSecondPerSecond)));

        currentDrawCalculatorSim.registerCurrentDraw(() -> statorCurrent);
    }

    @Override
    public void updateInputs(AngularIOInputs inputs) {
        armLength.ifPresent(length -> pivot.setArmLength(length.get()));

        inputs.goal = this.goal.orElse(Radians.of(0.0));
        switch (outputMode) {
            case kClosedLoop ->
                    inputs.appliedVolts =
                            Volts.of(
                                    MathUtil.clamp(
                                            controller.calculate(
                                                            pivot.getAngleRads(),
                                                            goal.orElse(Radians.of(0.0))
                                                                    .in(Radians))
                                                    * RobotController.getBatteryVoltage(),
                                            -12.0,
                                            12.0));
            case kOpenLoop ->
                    inputs.appliedVolts =
                            Volts.of(dutyCycle.orElse(0.0) * RobotController.getBatteryVoltage());
            case kNeutral -> inputs.appliedVolts = Volts.of(0.0);
        }
        pivot.setInput(inputs.appliedVolts.in(Volts));
        pivot.update(kDt);

        inputs.angle = Radians.of(pivot.getAngleRads());

        inputs.supplyCurrent = Amps.of(pivot.getCurrentDrawAmps());
        inputs.statorCurrent = Amps.of(pivot.getCurrentDrawAmps());
        this.statorCurrent = inputs.statorCurrent;

        inputs.velocity = RadiansPerSecond.of(pivot.getVelocityRadPerSec());
        this.velocity = inputs.velocity;
        inputs.acceleration = RadiansPerSecondPerSecond.of(0.0);

        inputs.motorTemperatures = this.motorTemperatures;
        inputs.deviceConnectedStatuses = this.deviceConnectedStatuses;

        inputs.neutralMode = deviceConfig.getNeutralMode();
        inputs.IOOutputMode = this.outputMode;
    }

    @Override
    public void setAngle(Angle angle) {
        controller.reset(angle.in(Radians), velocity.in(RadiansPerSecond));
        this.goal = Optional.of(angle);
        this.dutyCycle = Optional.empty();
        outputMode = kClosedLoop;
    }

    @Override
    public void setOpenLoop(double dutyCycle) {
        this.goal = Optional.empty();
        this.dutyCycle = Optional.of(dutyCycle);
        outputMode = kOpenLoop;
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
        outputMode = kNeutral;
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        deviceConfig.setKP(kP);
        deviceConfig.setKI(kI);
        deviceConfig.setKD(kD);

        controller.setPID(kP, kI, kD);
    }

    @Override
    public void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
        deviceConfig.setCruiseVelocity(cruiseVelocity);
        deviceConfig.setAcceleration(acceleration);

        controller.setConstraints(
                new TrapezoidProfile.Constraints(
                        cruiseVelocity.in(RadiansPerSecond),
                        acceleration.in(RadiansPerSecondPerSecond)));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        deviceConfig.setNeutralMode(neutralMode);
    }

    public void setRealAngleFromSubsystemAngleZeroSupplier(
            Supplier<Rotation2d> realAngleFromSubsystemAngleZero) {
        this.realAngleFromSubsystemAngleZero = Optional.ofNullable(realAngleFromSubsystemAngleZero);
    }

    public void setArmLengthSupplier(Supplier<Distance> length) {
        this.armLength = Optional.of(length);
    }
}
