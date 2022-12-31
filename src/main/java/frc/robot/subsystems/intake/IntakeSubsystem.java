package frc.robot.subsystems.intake;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private CANSparkMax rightMotorIndexer = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax intakeMotor = new CANSparkMax(7, MotorType.kBrushless);
	private Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4); //4 prime, 0 deux
    private DigitalInput prox1 = new DigitalInput(0);
    private boolean fastMode = false;

    public IntakeSubsystem() {
        rightMotorIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
        rightMotorIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        rightMotorIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        intakeMotor.burnFlash();
        rightMotorIndexer.burnFlash();
    }

    public void setFast(boolean mode) {
        fastMode = mode;
    }

    public boolean getFast() {
        return fastMode;
    }

    public void runIntakeMotor(){
        intakeMotor.set(-1.0);
    }
    public void stopIntakeMotor(){
        intakeMotor.set(0.0);
    }
    public void deployIntake() {
        intakeSolenoid.set(true);
    }

    public void setSpeed(double speed){
        rightMotorIndexer.set(speed);
    }
    public void retractIntake() {
        intakeSolenoid.set(false);
    }
    public void reverseIndexer()
    {
        rightMotorIndexer.set(-1.0);
        intakeMotor.set(1.0);
    }

    public void runRightMotor() {
        rightMotorIndexer.set(0.8);
    }

    public void runRightMotorShoot() {
        rightMotorIndexer.set(0.8);
    }

    public void runRightMotorShootFast() {
        rightMotorIndexer.set(0.8);
    }

    public void outtake() {
        intakeMotor.set(1.0);
        rightMotorIndexer.set(-1.0);
    }
    
    public void stopRightMotor() {
        rightMotorIndexer.set(0.0);
    }

    public boolean getProx1() {
        return prox1.get();
    }
    public boolean getSolenoid(){
        return intakeSolenoid.get();
    }
}
