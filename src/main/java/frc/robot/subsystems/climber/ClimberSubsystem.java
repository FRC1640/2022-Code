package frc.robot.subsystems.climber;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.commands.DriveClimber;
import edu.wpi.first.wpilibj.SPI;
public class ClimberSubsystem extends SubsystemBase {
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    private CANSparkMax liftMotor1 = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkMax liftMotor2 = new CANSparkMax(14, MotorType.kBrushless);
    private CANSparkMax liftMotor3 = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax liftMotor4 = new CANSparkMax(16, MotorType.kBrushless);
    private SparkMaxPIDController liftController;
    double processVariable;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private DoubleSolenoid tiltSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3); //2 3
    private Solenoid clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 5); //TODO this port is wrong maybe idk, make it less wrong

    private RelativeEncoder encoder = liftMotor4.getEncoder();

    public ClimberSubsystem() {

        liftMotor1.follow(liftMotor4);
        liftMotor2.follow(liftMotor4);
        liftMotor3.follow(liftMotor4);
        liftMotor4.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 50);
        liftMotor4.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        liftMotor4.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        liftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        liftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        liftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        liftMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        liftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        liftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        liftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        liftMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        liftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        liftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        liftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        liftMotor3.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        liftMotor1.burnFlash();
        liftMotor2.burnFlash();
        liftMotor3.burnFlash();
        liftMotor4.burnFlash();
        setupMotor();

    }
    public void setupMotor(){
        liftMotor1.restoreFactoryDefaults();
        liftController = liftMotor4.getPIDController();
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
        maxVel = 5500; // rpm
        maxAcc = 4500;
        liftController.setP(kP);
        liftController.setI(kI);
        liftController.setD(kD);
        liftController.setIZone(kIz);
        liftController.setFF(kFF);
        liftController.setOutputRange(kMinOutput, kMaxOutput);
        
        int smartMotionSlot = 0;
        liftController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        liftController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        liftController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        liftController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }
    public void quickSetup(){
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;
        maxVel = 5500; // rpm
        maxAcc = 4500;
        liftController.setP(kP);
        liftController.setI(kI);
        liftController.setD(kD);
        liftController.setIZone(kIz);
        liftController.setFF(kFF);
        liftController.setOutputRange(kMinOutput, kMaxOutput);
        
        int smartMotionSlot = 0;
        liftController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        liftController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        liftController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        liftController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }
    public void setupMotorSlow(){
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 2500;
        maxVel = 2500; // rpm
        maxAcc = 2000;
        liftController.setP(kP);
        liftController.setI(kI);
        liftController.setD(kD);
        liftController.setIZone(kIz);
        liftController.setFF(kFF);
        liftController.setOutputRange(kMinOutput, kMaxOutput);
        
        int smartMotionSlot = 0;
        liftController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        liftController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        liftController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        liftController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }
    public void setSpeed(double speed) {
        liftMotor4.set(speed);
    }
    public void motionProfile(double setPoint){
        //System.out.println(liftMotor1.getEncoder().getVelocity());
        liftController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariable = liftMotor4.getEncoder().getPosition();
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveClimber(this));
    }
    public void setMotors(double speed) {
        if((liftMotor4.getEncoder().getPosition() >= -129 && Math.signum(speed) < -0.2) || (liftMotor4.getEncoder().getPosition() <= 300 && Math.signum(speed) > 0.2)) {
            liftMotor4.set(speed);
        } else {
            liftMotor4.set(0);
        }
        // System.out.println("Executing");
        // liftMotor4.set(speed);
        
    }
    public boolean limit(){
        return liftMotor4.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public void setVertical(boolean state) {
        Value vState = (state) ? Value.kForward : Value.kReverse;
        tiltSolenoid.set(vState);
    }

    public double getPosition() {
        return encoder.getPosition();
    }
    public float getGyroRoll(){
        return gyro.getRoll();
    }

    public void disengageBrake() {
        clawSolenoid.set(false);
    }

    public void engageBrake() {
        clawSolenoid.set(true);
    }
    
}