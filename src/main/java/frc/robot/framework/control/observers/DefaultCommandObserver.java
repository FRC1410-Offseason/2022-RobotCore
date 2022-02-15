package frc.robot.framework.control.observers;

import frc.robotmap.IDs.SCHEDULER_PRIORITY;
 
public class DefaultCommandObserver extends Observer {
    
    public DefaultCommandObserver() {
        setPriority(SCHEDULER_PRIORITY.LOW);
    }
    
    public void check() {
        if (!task.isEnabled()) {
            requestExecution();
        }
    }
}
