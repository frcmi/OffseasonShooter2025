package team5937.lib.subsystem.angular;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amps;
import static team5937.lib.subsystem.angular.AngularIOOutputMode.kNeutral;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import team5937.lib.subsystem.DeviceConnectedStatus;

import org.littletonrobotics.junction.AutoLog;

public interface AngularIO {
    default void updateInputs(AngularIOInputs inputs) {}

    @AutoLog
    class AngularIOInputs {
        // Separate from AngularSubsystemOutputMode.
        public AngularIOOutputMode IOOutputMode = kNeutral;

        public Angle angle = Radians.of(0.0);

        public Voltage appliedVolts = Volts.of(0.0);
        public Current supplyCurrent = Amps.of(0.0);
        public Current statorCurrent = Amps.of(0.0);
        public AngularVelocity velocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0.0);

        public NeutralModeValue neutralMode = NeutralModeValue.Brake;

        public double[] motorTemperatures = {};
        public DeviceConnectedStatus[] deviceConnectedStatuses = {};

        public Angle goalPos = Radians.of(0.0);
        public AngularVelocity goalVel = RadiansPerSecond.of(0.0);
    }

    default void setAngle(Angle angle) {}

    default void setOpenLoop(Voltage voltage) {}

    default void setVelocity(AngularVelocity velocity) {}

    default void stop() {}

    default void resetAngle() {}

    default void resetAngle(Angle angle) {}

    default void setPIDV(double kP, double kI, double kD, double kV) {}

    default void setConstraints(AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {}

    default void setNeutralMode(NeutralModeValue neutralMode) {}

    default void setLogKey(String logKey) {}
}
