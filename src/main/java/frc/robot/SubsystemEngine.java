package frc.robot;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;

public class SubsystemEngine {

    //Singleton instance creation
    private static SubsystemEngine engineInstance;
    public static SubsystemEngine getInstance() {
        if (engineInstance == null) {
            engineInstance = new SubsystemEngine();
        }
        return engineInstance;
    }

    public void refreshInstance() {
        rawSubsystemMap.clear();
    }

    //Subsystem table handling
    private Map<Class, Object> rawSubsystemMap = new HashMap<Class, Object>();
    public <T> SubsystemBase getSubsystem(Class<T> subsystemKey) {
        SubsystemBase returnInstance = null;

        try {
            if (!getInstance().rawSubsystemMap.containsKey(subsystemKey)) {
                T obj = subsystemKey.newInstance();
                getInstance().rawSubsystemMap.put(subsystemKey, obj);

                try {
                    SubsystemBase castedSubsystem = (SubsystemBase) obj;
                } catch (Exception e) {
                    //Future TelemetryHandler warning for "Improper Subsystem Cast"
                }
            }
            returnInstance = (SubsystemBase) getInstance().rawSubsystemMap.get(subsystemKey);
        } catch (Exception e) {
            //Future Telemetry Post for "Cannot Instantiate"
        }

        return returnInstance;
    }
}