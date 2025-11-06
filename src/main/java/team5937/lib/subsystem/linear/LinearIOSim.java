package team5937.lib.subsystem.linear;

import static edu.wpi.first.units.Units.*;
import static team5937.frc2025.constants.RobotConstants.kDt;
import static team5937.lib.subsystem.linear.LinearIOOutputMode.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import team5937.lib.sim.CurrentDrawCalculatorSim;
import team5937.lib.sim.LinearExtensionSim;
import team5937.lib.subsystem.DeviceConnectedStatus;

import java.util.Optional;
import java.util.function.Supplier;

public class LinearIOSim implements LinearIO {
    private final LinearExtensionSim linearExtension;
    private final ProfiledPIDController controller;

    private final LinearIOSimConfig deviceConfig;

    private final double[] motorTemperatures = new double[] {};
    private final DeviceConnectedStatus[] deviceConnectedStatuses = new DeviceConnectedStatus[] {};

    private LinearIOOutputMode outputMode = kNeutral;
    private Optional<Distance> goal = Optional.empty();
    private Optional<Double> dutyCycle = Optional.empty();

    private Current statorCurrent = Amps.of(0.0);

    private Optional<Supplier<Rotation2d>> angle = Optional.empty();

    private LinearVelocity velocity = MetersPerSecond.of(0.0);

    public LinearIOSim(
            LinearIOSimConfig config, CurrentDrawCalculatorSim currentDrawCalculatorSim) {
        this.deviceConfig = config;

        // Hardware
        DCMotor motor = config.getMotor();
        linearExtension =
                new LinearExtensionSim(
                        motor,
                        config.getMotorRotationsPerOutputRotations(),
                        config.getCarriageMass().in(Kilograms),
                        config.getOutputDistancePerOutputRotation().div(Math.PI).in(Meters),
                        config.getPhysicalMinLength().in(Meters),
                        config.getPhysicalMaxLength().in(Meters),
                        angle,
                        config.getResetLength().in(Meters));
        controller =
                new ProfiledPIDController(
                        config.getKP(),
                        config.getKI(),
                        config.getKD(),
                        new TrapezoidProfile.Constraints(
                                config.getCruiseVelocity().in(MetersPerSecond),
                                config.getAcceleration().in(MetersPerSecondPerSecond)));

        currentDrawCalculatorSim.registerCurrentDraw(() -> statorCurrent);
    }

    @Override
    public void updateInputs(LinearIOInputs inputs) {
        inputs.goal = this.goal.orElse(Meters.of(0.0));
        switch (outputMode) {
            case kClosedLoop ->
                    inputs.appliedVolts =
                            Volts.of(
                                    MathUtil.clamp(
                                            controller.calculate(
                                                            linearExtension.getPositionMeters(),
                                                            goal.orElse(Meters.of(0.0)).in(Meters))
                                                    * RobotController.getBatteryVoltage(),
                                            -12.0,
                                            12.0));
            case kOpenLoop ->
                    inputs.appliedVolts =
                            Volts.of(dutyCycle.orElse(0.0) * RobotController.getBatteryVoltage());
            case kNeutral -> inputs.appliedVolts = Volts.of(0.0);
        }
        linearExtension.setInput(inputs.appliedVolts.in(Volts));
        linearExtension.update(kDt);

        inputs.length = Meters.of(linearExtension.getPositionMeters());

        inputs.supplyCurrent = Amps.of(linearExtension.getCurrentDrawAmps());
        inputs.statorCurrent = Amps.of(linearExtension.getCurrentDrawAmps());
        this.statorCurrent = inputs.statorCurrent;

        inputs.velocity = MetersPerSecond.of(linearExtension.getVelocityMetersPerSecond());
        this.velocity = inputs.velocity;
        inputs.acceleration = MetersPerSecondPerSecond.of(0.0);

        inputs.motorTemperatures = this.motorTemperatures;
        inputs.deviceConnectedStatuses = this.deviceConnectedStatuses;

        inputs.neutralMode = deviceConfig.getNeutralMode();
        inputs.IOOutputMode = this.outputMode;
    }

    @Override
    public void setLength(Distance length) {
        controller.reset(length.in(Meters), velocity.in(MetersPerSecond));
        this.goal = Optional.of(length);
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
    public void setConstraints(LinearVelocity cruiseVelocity, LinearAcceleration acceleration) {
        deviceConfig.setCruiseVelocity(cruiseVelocity);
        deviceConfig.setAcceleration(acceleration);

        controller.setConstraints(
                new TrapezoidProfile.Constraints(
                        cruiseVelocity.in(MetersPerSecond),
                        acceleration.in(MetersPerSecondPerSecond)));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        deviceConfig.setNeutralMode(neutralMode);
    }

    public void setAngleSupplier(Supplier<Rotation2d> angleSupplier) {
        this.angle = Optional.ofNullable(angleSupplier);
    }
}
