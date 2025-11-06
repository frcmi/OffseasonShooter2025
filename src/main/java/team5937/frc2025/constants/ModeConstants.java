package team5937.frc2025.constants;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public class ModeConstants {
    public static final Mode kSimMode = Mode.kSim;
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.kReal : kSimMode;

    public static final boolean kTuningMode = true;

    public static enum Mode {
        /** Running on a real robot. */
        kReal,

        /** Running a physics simulator. */
        kSim,

        /** Replaying from a log file. */
        kReplay
    }
}
