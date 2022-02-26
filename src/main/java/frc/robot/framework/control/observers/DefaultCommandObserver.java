package frc.robot.framework.control.observers;

import frc.robotmap.IDs.SCHEDULER_PRIORITY;
 
public class DefaultCommandObserver extends Observer {
    
    public DefaultCommandObserver() {
        configurePriority(SCHEDULER_PRIORITY.LOW);
    }
    
    public void check() {
        requestExecution();
    }
}
