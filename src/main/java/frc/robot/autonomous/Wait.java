package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {

    private long initTime = 0;
    private long duration = 0;

    public Wait(long duration) {
        this.duration = duration;
    }

    @Override
    public void initialize() {
        initTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - initTime >= duration;
    }
    
}
