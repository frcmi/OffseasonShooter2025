package team5937.lib.alliancecolor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team5937.lib.subsystem.VirtualSubsystem;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class AllianceChecker extends VirtualSubsystem {
    private final List<AllianceUpdatedObserver> observers = new ArrayList<>();
    private Optional<Alliance> alliance = DriverStation.getAlliance();

    public void registerObserver(AllianceUpdatedObserver observer) {
        observers.add(observer);
    }

    public void registerObservers(AllianceUpdatedObserver... addObservers) {
        Collections.addAll(observers, addObservers);
    }

    public void periodic() {
        alliance = DriverStation.getAlliance();

        alliance.ifPresent(color -> observers.forEach(observer -> observer.onAllianceFound(color)));
    }
}
