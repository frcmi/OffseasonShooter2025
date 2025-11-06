package team5937.lib.alliancecolor;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public interface AllianceUpdatedObserver {
    void onAllianceFound(Alliance alliance);
}
