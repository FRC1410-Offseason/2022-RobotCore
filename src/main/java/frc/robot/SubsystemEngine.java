package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

import java.util.*;

public class SubsystemEngine {
    private static final SubsystemEngine instance = new SubsystemEngine();

    public static SubsystemEngine getInstance() {
        return instance;
    }

    public void refreshInstance() {
        rawSubsystemMap.clear();
    }

    //Subsystem table handling
    private final Map<Class<? extends Subsystem>, Object> rawSubsystemMap = new IdentityHashMap<>();

    public <T extends Subsystem> T getSubsystem(Class<T> type) {
        @SuppressWarnings("unchecked")
        T subsystem = (T) rawSubsystemMap.get(type);

        if (subsystem == null) {
            try {
                subsystem = type.getDeclaredConstructor().newInstance();
            } catch (ReflectiveOperationException e) {
                DriverStation.reportError("Error instantiating subsystem!", e.getStackTrace());
            }

            rawSubsystemMap.put(type, subsystem);
        }

        return subsystem;
    }
}
