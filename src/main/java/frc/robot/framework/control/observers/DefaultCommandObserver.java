package frc.robot.framework.control.observers;

import frc.robotmap.IDs.OBSERVER_PRIORITY;
 
public class DefaultCommandObserver extends Observer {
    
    public DefaultCommandObserver() {
        setPriority(OBSERVER_PRIORITY.LOW);
    }
    
    public void check() {
        if (!task.isEnabled()) {
            
        }
    }
}
