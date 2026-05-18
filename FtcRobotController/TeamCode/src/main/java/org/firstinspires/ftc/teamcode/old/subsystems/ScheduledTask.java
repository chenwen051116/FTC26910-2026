package org.firstinspires.ftc.teamcode.old.subsystems;

public class ScheduledTask {
    public double milliseconds;  // the time to execute the task (in milliseconds)
    public Runnable task;

    public ScheduledTask(double time, Runnable task) {
        this.milliseconds = time;
        this.task = task;
    }
}
