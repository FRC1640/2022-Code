package frc.robot.subsystems.shooter.commands;

import java.applet.AudioClip;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Limelight;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.Drive;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AlignTurret extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private DriveSubsystem driveSubsystem;
    private XboxController controller = new XboxController(1);
    private XboxController controllerDriver = new XboxController(0);
    private boolean stopTurret = true;

    public AlignTurret(ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        shooterSubsystem.ledOn();
    }

    @Override
    public void execute() {
        // System.out.println("Distance: " + shooterSubsystem.distance());
        // System.out.println("Turret: " + shooterSubsystem.getTurretPosition());
        //System.out.println("Gyro: " + driveSubsystem.getPose().getRotation());
        // System.out.print("Area: " + shooterSubsystem.getArea());
        // System.out.println("Limelight: " + shooterSubsystem.findLimelightError());
        // System.out.println("RightBumper: " + controller.getRightBumper() + " Area: " + (shooterSubsystem.getArea() != 0));
        // System.out.println("Limelight: " + shooterSubsystem.findLimelightError());
        // System.out.println("X: " + controller.getRightX());
        // System.out.println("Y: " + controller.getRightY());
        //System.out.println("Turret: " + shooterSubsystem.getTurretPosition());
        double joystickAngle = Math.toDegrees(Math.atan2(-controller.getLeftX(), -controller.getLeftY()));
        //System.out.println("Gyro: " + shooterSubsystem.getGyroAngle());
        double adjustedAngle = (((joystickAngle) - shooterSubsystem.getGyroAngle()) + 180) % 360;
        if (adjustedAngle >= 180){ //270, -90
            adjustedAngle -= 360;
        }
        // System.out.println("Encoder Actual: " + shooterSubsystem.getEncoder());
        if(controller.getRightTriggerAxis() > 0.1) {
            
            shooterSubsystem.pointTurretController(0.5);
        }
        else if(controller.getLeftTriggerAxis() > 0.1) {
            shooterSubsystem.pointTurretController(-0.5);   
        }
        else if (shooterSubsystem.getArea() == 0 && !stopTurret){ //
            Pose2d pose = driveSubsystem.getPose();
            double xOffset =  8.2 - pose.getX();
            double yOffset = 4.1 - pose.getY();
            double angle = Math.toDegrees((Math.atan2(yOffset,xOffset)));
            //(0.57, 2.54)
            //System.out.println("Angle: " + angle);
            double newAngle = ((angle - shooterSubsystem.getGyroAngle()) + 180) % 360;
            if (newAngle >= 180){
                newAngle -= 360;
            }
            controller.setRumble(RumbleType.kLeftRumble, 0);
            //System.out.println("New Angle: " + newAngle);
            shooterSubsystem.pointTurretPosition(((-newAngle * 0.77288) + 80.389));
        }
        else if ((Math.abs(controller.getLeftX()) >= 0.5 || Math.abs(controller.getLeftY()) >= 0.5 && shooterSubsystem.getArea() == 0)){// 
            
            // System.out.println("Angle: " + ((joystickAngle * 0.77288) + 80.389));
            // 
            // System.out.println("Angle: " + joystickAngle);
            // System.out.println("Adjusted: " + adjustedAngle);
            // System.out.println("Encoder: " + adjustedAngle * 1.5);
            // System.out.print("Angle but actually this time: " + ((adjustedAngle * 0.77288) + 80.389));
            shooterSubsystem.pointTurretPosition(((-adjustedAngle * 0.77288) + 80.389));
            
        }
        else{
            if(stopTurret == false) {
                shooterSubsystem.pointTurret();
            } else {
                shooterSubsystem.pointTurret();
            }
        }
        //System.out.println("Encoder: " + shooterSubsystem.getEncoder());
        if(controller.getStartButtonPressed()) {
            shooterSubsystem.resetOffset();
        }

        if(controller.getBackButtonPressed()) {
            shooterSubsystem.resetVelocityOffset();
        }
        // if (stopTurret){
            
        //     controller.setRumble(RumbleType.kLeftRumble, 0.3);
        // }
        // else{
        //     controller.setRumble(RumbleType.kLeftRumble, 0);
        // }
        // System.out.println(shooterSubsystem.)

        // if(controller.getLeftBumperPressed()) {
        //     shooterSubsystem.resetHeadingAngle();
        // }

        if(controllerDriver.getYButtonPressed()) {
            stopTurret = !stopTurret;
        }
        Button up = new Button() {
            public boolean get() {
                return controller.getPOV() == 0;
            }
        };
        Button down = new Button() {
            public boolean get() {
                return controller.getPOV() == 180;
            }
        };

        Button left = new Button() {
            public boolean get() {
                return controller.getPOV() == 90;
            }
        };

        Button right = new Button() {
            public boolean get() {
                return controller.getPOV() == 270;
            }
        };

        Button rightDriver = new Button() {
            public boolean get() {
                return controllerDriver.getPOV() == 270;
            }
        };

        // if(down.get()) {
        //     shooterSubsystem.setTrajectoryAngle(0.125);
        // }
        // else if(up.get()) {
        //     shooterSubsystem.setTrajectoryAngle(-0.125);
        // }
        if (up.get()){
            stopTurret = !stopTurret;
        }
        shooterSubsystem.motionProfileAngleCalculated();
        shooterSubsystem.setTargetRpm(0);
        shooterSubsystem.setMotorSpeeds(0.2, 0.3);
        shooterSubsystem.setPipeline();
        // System.out.println(shooterSubsystem.getDirection());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
