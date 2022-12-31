package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.commands.Index;

public class IndexerSubsystem extends SubsystemBase {
    private CANSparkMax leftMotorIndexer = new CANSparkMax(4, MotorType.kBrushless);
    private DigitalInput prox2 = new DigitalInput(1);
    private int ballCount = 0;
    private boolean fastMode = false;

    public IndexerSubsystem() {
        leftMotorIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
        leftMotorIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        leftMotorIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        leftMotorIndexer.burnFlash();
    }

    public void initDefaultCommand() {
        setDefaultCommand(new Index(this));
    }

    public void setFast(boolean mode) {
        fastMode = mode;
    }

    public boolean getFast() {
        return fastMode;
    }

    public void runUppererMotor() {
        leftMotorIndexer.set(0.8);
    }

    public void runUppererMotorShoot() {
        leftMotorIndexer.set(0.8);
    }

    public void runUppererMotorShootFast() {
        leftMotorIndexer.set(0.8);
    }

    public void reverseIndexer() {
        leftMotorIndexer.set(-1.0);
    }

    public void stopIndexer() {
        leftMotorIndexer.set(0.0);
    }

    public void setBallCount(int ballCount) {
        this.ballCount = ballCount;
    }

    public int getBallCount() {
        return ballCount;
    }
    public boolean getProx2() {
        return prox2.get();
    }
}
