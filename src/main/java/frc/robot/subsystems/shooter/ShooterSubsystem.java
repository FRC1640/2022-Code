package frc.robot.subsystems.shooter;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.Limelight.LedEnum;
import frc.robot.subsystems.shooter.commands.AlignTurret;

public class ShooterSubsystem extends SubsystemBase {
    private double setAngle;
    private CANSparkMax shooterMotor1 = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax shooterMotor2 = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax turretMotor = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax angleMotor = new CANSparkMax(19, MotorType.kBrushless);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    Limelight limelight = new Limelight();

    private double[] angles = {21, 111, 201, 291};

    private PIDController turretPid = new PIDController(0.035, 0.0, 0.0);

    double targetRpm = 0;
    double targetAngle = 0;
    double targetHeading = 0;
    double equationOffset = 0;
    double velocityOffset = 0;

    private SparkMaxPIDController shooterController;
    private SparkMaxPIDController shooterController2;
    private SparkMaxPIDController angleController;
    private SparkMaxPIDController turretController;
    private SparkMaxPIDController turretControllerSlow;
    double processVariableAngle;
    double processVariableTurret;
    double processVariable;
    double processVariable2;
    double var1;
    double var2;
    double var3;
    double var4;
    double motor1Setpoint;
    double motor2Setpoint;
    double offset;
    boolean withinDistance;
    int count;
    public ShooterSubsystem() {
        // shooterMotor2.follow(shooterMotor1);
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        shooterMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        shooterMotor2.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        turretMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        turretMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);
        setupMotorSpeed();
        setupMotorAngle();
        setupMotorPointSlow();
        setupMotorPoint();
        shooterMotor1.burnFlash();
        shooterMotor2.burnFlash();
        turretMotor.burnFlash();
        angleMotor.burnFlash();
    }

    public double getTurretPosition(){
        return turretMotor.getEncoder().getPosition();
    }
    public void setupMotorSpeed() {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
        allowedErr = 0;
        minVel = 0;
        shooterMotor1.restoreFactoryDefaults();
        // liftEncoder = liftMotor.getEncoder();
        shooterController = shooterMotor1.getPIDController();
        kP = 0.00013; //0.00013
        kI = 0.0;
        kD = 0.0005; //0.0005
        kIz = 0; 
        kFF = 0.0002; //0.0002
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5200;
        maxVel = 5200; // rpm
        maxAcc = 4500;
        shooterController.setP(kP);
        shooterController.setI(kI);
        shooterController.setD(kD);
        shooterController.setIZone(kIz);
        shooterController.setFF(kFF);
        shooterController.setOutputRange(kMinOutput, kMaxOutput);
        
        int smartMotionSlot = 0;
        shooterController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        shooterController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        shooterController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        shooterController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    }

    public void setOffset(double value) {
        offset = value;
    }

    public double getOffset() {
        return offset;
    }

    public double getGyroAngle(){
        //return gyro.getRotation2d().getDegrees() - Math.toDegrees(getOffset())
        return gyro.getRotation2d().getDegrees() - getOffset();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double getArea(){
        return limelight.getTargetArea();
    }
    public void setupMotorSpeed2() {
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
        allowedErr = 0;
        minVel = 0;
        shooterMotor2.restoreFactoryDefaults();
        // liftEncoder = liftMotor.getEncoder();
        shooterController2 = shooterMotor2.getPIDController();
        kP = 0.00013; //0.00013
        kI = 0.0;
        kD = 0.0005; //0.0005
        kIz = 0; 
        kFF = 0.0002; //0.0002
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5200;
        maxVel = 5200; // rpm
        maxAcc = 4500;
        shooterController2.setI(kI);
        shooterController2.setP(kP);
        shooterController2.setD(kD);
        shooterController2.setIZone(kIz);
        shooterController2.setFF(kFF);
        shooterController2.setOutputRange(kMinOutput, kMaxOutput);
        
        int smartMotionSlot = 0;
        shooterController2.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        shooterController2.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        shooterController2.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        shooterController2.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    }
    public void setupMotorAngle(){
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
        allowedErr = 0;
        minVel = 0;
        angleMotor.restoreFactoryDefaults();
        // liftEncoder = liftMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        kP = 0.016; 
        kI = 0.0;
        kD = 0.00001; 
        kIz = 0; 
        kFF = 0.0004; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5200;
        maxVel = 5200; // rpm
        maxAcc = 4500;

        angleController.setP(kP);
        angleController.setI(kI);
        angleController.setD(kD);
        angleController.setIZone(kIz);
        angleController.setFF(kFF);
        angleController.setOutputRange(kMinOutput, kMaxOutput);
        
        int smartMotionSlot = 0;
        angleController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        angleController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        angleController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        angleController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }

    public void setupMotorPoint(){
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
        allowedErr = 0;
        minVel = 0;
        
        // liftEncoder = liftMotor.getEncoder();
        turretController = turretMotor.getPIDController();
        kP = 0.035;
        kI = 0.0;
        kD = 0.02; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5200;
        maxVel = 4500; // rpm
        maxAcc = 4500;

        turretController.setP(kP);
        turretController.setI(kI);
        turretController.setD(kD);
        turretController.setIZone(kIz);
        turretController.setFF(kFF);
        turretController.setOutputRange(kMinOutput, kMaxOutput);
        
        int smartMotionSlot = 0;
        turretController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        turretController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        turretController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        turretController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }
    public void setupMotorPointSlow(){
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
        allowedErr = 0;
        minVel = 0;
        
        // liftEncoder = liftMotor.getEncoder();
        turretControllerSlow = turretMotor.getPIDController();
        kP = 0.0245;
        kI = 0.0;
        kD = 0.018; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5200;
        maxVel = 5200; // rpm
        maxAcc = 4500;

        turretControllerSlow.setP(kP);
        turretControllerSlow.setI(kI);
        turretControllerSlow.setD(kD);
        turretControllerSlow.setIZone(kIz);
        turretControllerSlow.setFF(kFF);
        turretControllerSlow.setOutputRange(kMinOutput, kMaxOutput);
        
        int smartMotionSlot = 0;
        turretControllerSlow.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        turretControllerSlow.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        turretControllerSlow.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        turretControllerSlow.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    }

    
    public double getClosestAngle(double[] arr, double currentAngle) {
        double closestNumber = arr[0];

            for (double num : arr) {
                if (Math.abs(num - currentAngle) < Math.abs(closestNumber - currentAngle)) {
                    closestNumber = num;
                }
            }
            return closestNumber;
    }

    public void increaseOffset() {
        equationOffset += 0.1;
        System.out.println("increment " + equationOffset);
    }

    public void decreaseOffset() {
        equationOffset -= 0.1;
        System.out.println("increment " + equationOffset);
    }

    public void increaseOffsetVelocity() {
        velocityOffset -= 0.001;
        System.out.println("velocity increment " + velocityOffset);
    }

    public void decreaseOffsetVelocity() {
        velocityOffset += 0.001;
        System.out.println("velocity increment " + velocityOffset);
    }

    public void resetHeadingAngle() {
        // turretMotor.getEncoder().setPosition(0);
        // angleMotor.getEncoder().setPosition(0);
    }

    public void resetOffset() {
        equationOffset = 0;
    }

    public void resetVelocityOffset() {
        velocityOffset = 0;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getTargetAngle() {
        return setAngle;
    }

    public void setTargetRpm(double value) {
        targetRpm = value;
    }

    public double getActualRpm() {
        return shooterMotor2.getEncoder().getVelocity();
    }

    public boolean atTargetRpm(){
        return Math.abs(getTargetRpm()) <= Math.abs(getActualRpm()) && getTargetRpm() != 0.0;
    }

    public boolean atTargetAngle(){
        // System.out.println("Current Angle " + angleMotor.getEncoder().getPosition());
        return angleMotor.getEncoder().getPosition() >= setAngle - 5 && angleMotor.getEncoder().getPosition() <= setAngle + 5;
    }

    public boolean pointedAtTarget() {
        // System.out.println("Current Heading " + turretMotor.getEncoder().getPosition());
        // System.out.println("Target Heading " + targetHeading);
        return turretMotor.getEncoder().getPosition() >= targetHeading - 5 && turretMotor.getEncoder().getPosition() <= targetHeading + 5;
    }

    public void setMotorSpeeds(double speed, double speed2) {
        shooterMotor1.set(speed);
        shooterMotor2.set(speed2);
    }

    public void setMotorSpeedsCalculated() {
        double distance = calculateDistanceFromTarget(limelight.getTargetY() + 25, 31, 104);
        double speed1 = 0.29*Math.pow(Math.E, 0.0022*distance); //0.000718403*distance + 0.379355 - 0.03
        // setTargetRpm(speed1*5300 - 400);
        // shooterMotor1.set(speed1 - 0.08);
        // shooterMotor2.set(speed1);
        motionProfile2((speed1 + 0.08)*4900);
        motionProfile(speed1*4900);
    }

    public void motionProfile(double setPoint){
        shooterController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        processVariable = shooterMotor1.getEncoder().getVelocity();
        motor1Setpoint = setPoint;
    }
    public void motionProfile2(double setPoint){
        shooterController2.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        processVariable2 = shooterMotor2.getEncoder().getVelocity();
        motor2Setpoint = setPoint;
    }
    public void motionProfileAngle(double setPoint){ 
        angleController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        processVariableAngle = angleMotor.getEncoder().getPosition();
        setAngle = setPoint;
    }
    public void motionProfileShooter(double setpointPercent){
        double setPoint = setpointPercent * 5700;
        setPoint = Math.min(0, setPoint);
        setPoint = Math.max(setPoint, -145);
        shooterController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariableAngle = angleMotor.getEncoder().getVelocity();
    }
    public void motionProfileAngleClosest(){
        double setPoint = getClosestAngle(angles, gyro.getYaw());
        angleController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariableAngle = angleMotor.getEncoder().getPosition();
        setAngle = setPoint;
    }
    public void motionProfileAngleCalculated() {
        double distance = calculateDistanceFromTarget(limelight.getTargetY() + 25, 31, 104);
        double setPoint = (-0.505663*distance) - 16.9834; //0.00293486*Math.pow(distance, 2) - 1.40956*distance + 52.7853
        setPoint = Math.min(0, setPoint);
        setPoint = Math.max(setPoint, -145);
        angleController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        processVariableAngle = angleMotor.getEncoder().getPosition();
        setAngle = setPoint;
    }
    public double getAngle() {
        return angleMotor.getEncoder().getPosition();
    }


    public void setTrajectoryAngle(double speed){
        if((angleMotor.getEncoder().getPosition() >= -135 && Math.signum(speed) < 0) || (angleMotor.getEncoder().getPosition() <= 0 && Math.signum(speed) > 0)) {
            angleMotor.set(speed);
        } else {
            angleMotor.set(0);
        }
    }
    public double findLimelightError(){
        return limelight.getTargetX(0);
    }
    public double getEncoder(){
        return turretMotor.getEncoder().getPosition();
    }
    public boolean turretLimit0(){
        return getTurretPosition() >= 190;
    }
    public boolean turretLimit1(){
        return getTurretPosition() <= -25;
    }
    public void pointTurretPosition(double position) {
        if (position >= 190){
            turretController.setReference(190, CANSparkMax.ControlType.kPosition);
            targetHeading = position;
        }
        else if (position <= -25){
            turretController.setReference(-25, CANSparkMax.ControlType.kPosition);
            targetHeading = position;
        }
        else {
            turretController.setReference(position, CANSparkMax.ControlType.kPosition);
            targetHeading = position;
        }
    }
    public void pointTurretPositionSlow(double position){
        if (position >= 190){
            turretControllerSlow.setReference(190, CANSparkMax.ControlType.kPosition);
            targetHeading = position;
        }
        else if (position <= -25){
            turretControllerSlow.setReference(-25, CANSparkMax.ControlType.kPosition);
            targetHeading = position;
        }
        else {
            turretControllerSlow.setReference(position, CANSparkMax.ControlType.kPosition);
            targetHeading = position;
        }
    }
    public void pointTurret() {
        // double error = turretPid.calculate(limelight.getTargetX(0), 0);
        double encoder = (limelight.getTargetX(0) * 0.88);
        double encoderSlow = (limelight.getTargetX(0) * 0.88);
        //System.out.println("Distance: " + calculateDistanceFromTarget(limelight.getTargetY() + 25, 31, 104));
        // System.out.println("Number: " + encoder);
        // System.out.println("Limelight: " + limelight.getTargetX(0));
        // System.out.println("Turret Position: " + getTurretPosition());
        // System.out.println("Target: " + (encoder + getTurretPosition()));
        if (Math.abs(limelight.getTargetX(0))> 0.2){
            setupMotorPoint();
            pointTurretPosition(encoder + getTurretPosition());
            //System.out.println("Using Fast PID");
        }
        else{
            setupMotorPointSlow();
            pointTurretPositionSlow(encoderSlow + getTurretPosition());
            //System.out.println("Using Slow PID");
        }
        
        //System.out.println("Encoder new: " + (encoder + turretMotor.getEncoder().getPosition()));
            //if great than 80, but moving left - okay
            //great than 80, moving right - no
            //less than 80, moving right - okay
            //less than 80, moving left - no
            //if(between -80 and 80 || greater than 80)
                // if ((getTurretPosition() <= 190 && getTurretPosition() >= -25) || (getTurretPosition() >= 190 && Math.signum(-error) < 0) || (getTurretPosition() <= -25 && Math.signum(-error) > 0)){
                //     if (Math.abs(limelight.getTargetX(0)) >= 11){
                //         if (Math.signum(limelight.getTargetX(0)) > 0){
                //             turretMotor.set(0.6);
                //         }
                //         else{
                //             turretMotor.set(-0.6);
                //         }
                //     }
                //     else{
                //         turretMotor.set(-error);
                //     }
                // }
                // else{
                //     turretMotor.set(0);
                // }
        
        // if((getTurretPosition() <= 80 && Math.signum(limelight.getTargetX(0)) > 0) || (getTurretPosition() >= -80 && Math.signum(limelight.getTargetX(0)) < 0)) { //Prime 358, -32 Deux 95, -100

        // } else {
        //     turretMotor.set(0);
        // }
        targetHeading = (limelight.getTargetX(0)* 2) + turretMotor.getEncoder().getPosition();

        // var1 = var2;
        // var2 = var3;
        // var3 = var4;
        // var4 = limelight.getTargetX(0);
        // double target = limelight.getTargetX(0);
        
        // if((getDirection() <= 358 && Math.signum(error) > 0) || (getDirection() >= -32 && Math.signum(error) < 0)) {
        //     targetHeading = (target * 2) + turretMotor.getEncoder().getPosition();
        //     turretController.setReference(targetHeading, CANSparkMax.ControlType.kSmartMotion);
            
        // } else {
        //     turretMotor.set(0);
        // }
    }

    public void pointTurretDeadReckoning(double position) {

        // position = Math.max(-32, position);
        // position = Math.min(358, position);

        if((getTurretPosition() <= 190 && position >= getTurretPosition()) || (getTurretPosition() >= -25 && position <= getTurretPosition())) { //Prime 358 -32
            if(position <= turretMotor.getEncoder().getPosition() - 5 || position >= turretMotor.getEncoder().getPosition() + 5) {
                if(position > turretMotor.getEncoder().getPosition()) {
                    turretMotor.set(1.0);
                }
                else if(position < turretMotor.getEncoder().getPosition()) {
                    turretMotor.set(-1.0);
                }
            } else {
                turretMotor.set(0);
            }
        } else {
            turretMotor.set(0);
        }

        // if(position <= turretMotor.getEncoder().getPosition() - 5 || position >= turretMotor.getEncoder().getPosition() + 5) {
        //     if(position > turretMotor.getEncoder().getPosition()) {
        //         turretMotor.set(1.0);
        //     }
        //     else if(position < turretMotor.getEncoder().getPosition()) {
        //         turretMotor.set(-1.0);
        //     }
        // } else {
        //     turretMotor.set(0);
        // }

        targetHeading = position;

    }

    

    public void pointTurretController(double speed) {
        if((getTurretPosition() <= 190 && Math.signum(speed) > 0) || (getTurretPosition() >= -25 && Math.signum(speed) < 0)) { //DONT CHANGE THESE
            turretMotor.set(speed);
        } else {
            turretMotor.set(0);
        }
    }
    public void initAverage(){
        var1 = 0;
        var2 = 0;
        var3 = 0;
        var4 = 0;
    }
    public void ledOn() {
        limelight.setLEDOn(LedEnum.FORCE_ON);
        limelight.setProcessing(true);
    }

    public void ledOff() {
        limelight.setLEDOn(LedEnum.FORCE_OFF);
        limelight.setProcessing(false);
    }
    public double distance(){
        return calculateDistanceFromTarget(limelight.getTargetY() + 25, 31, 104);
    }
    private float calculateDistanceFromTarget(double angleDegrees, double distanceToFloorInches, // limelight.getTargetY() + 25, 31, 104
            double heightOfTargetInches) {
        return (float) ((heightOfTargetInches - distanceToFloorInches) / (Math.tan(Math.toRadians(angleDegrees))));
    }

    public void setPipeline() {
        // if(calculateDistanceFromTarget(limelight.getTargetY() + 25, 31, 104) <= 210) {
        //     limelight.setPipeline(0);
        // } else {
        //     limelight.setPipeline(1);
        // }
    }
    public boolean atTurretAngle(){
        return Math.abs(limelight.getTargetX(0)) <= 2;
    }
    public void logShot(){
        System.out.println("Current time: " + System.currentTimeMillis() + "\nLimelight distance: " + calculateDistanceFromTarget(limelight.getTargetY() + 28, 31, 104) +
        "\nLimelight error: " + limelight.getTargetX(0) + "\nLimelight average: " + (var1 + var2 + var3 + var4) / 4 + "\nTurret actual: " + turretMotor.getEncoder().getPosition() + "\nTurret target: " + 
        (((var1 + var2 + var3 + var4) / 4) * 2) + turretMotor.getEncoder().getPosition() + "\nHood actual: " + angleMotor.getEncoder().getPosition() +
        "\nHood target: " + setAngle + "\nMotor 1 speed: " + shooterMotor1.getEncoder().getVelocity() + "\nMotor 2 speed: " + shooterMotor2.getEncoder().getVelocity() + 
        "\nMotor 1 target: " + motor1Setpoint + "\nMotor 2 target: " + motor2Setpoint);

    }
}
