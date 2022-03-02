package frc.robot.framework.control.observers;

import frc.robotmap.IDs.SchedulerPriority;
 
public class DefaultCommandObserver extends Observer {
    
    public DefaultCommandObserver() {
        configurePriority(SchedulerPriority.LOW);
    }
    
    public void check() {
        requestExecution();
    }
}
