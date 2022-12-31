package frc.robot.subsystems.shooter.commands;

import java.applet.AudioClip;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AlignTurretOld extends CommandBase {

    private ShooterSubsystem shooterSubsystem;
    private XboxController controller = new XboxController(1);
    private XboxController controllerDriver = new XboxController(0);
    private boolean stopTurret = true;

    public AlignTurretOld(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {
        shooterSubsystem.ledOn();
    }

    @Override
    public void execute() {
        //System.out.println("Turret: " + shooterSubsystem.getTurretPosition());
        // System.out.print("Area: " + shooterSubsystem.getArea());
        // System.out.println("Limelight: " + shooterSubsystem.findLimelightError());
        // System.out.println("RightBumper: " + controller.getRightBumper() + " Area: " + (shooterSubsystem.getArea() != 0));
        // System.out.println("Limelight: " + shooterSubsystem.findLimelightError());
        // System.out.println("X: " + controller.getRightX());
        // System.out.println("Y: " + controller.getRightY());
        //System.out.println("Turret: " + shooterSubsystem.getTurretPosition());
        double joystickAngle = Math.toDegrees(Math.atan2(controller.getLeftX(), -controller.getLeftY())) + 180;
        
        double adjustedAngle = (joystickAngle + (shooterSubsystem.getGyroAngle())) % 360 + 163;
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

        else if ((Math.abs(controller.getLeftX()) >= 0.5 || Math.abs(controller.getLeftY()) >= 0.5) && shooterSubsystem.getArea() == 0){
            
            // System.out.println("Angle: " + ((joystickAngle * 0.77288) + 80.389));
            // System.out.println("Gyro: " + shooterSubsystem.getGyroAngle());
            // System.out.println("Angle: " + joystickAngle);
            // System.out.println("Adjusted: " + adjustedAngle);
            // System.out.println("Encoder: " + adjustedAngle * 1.5);
            // System.out.print("Angle but actually this time: " + ((adjustedAngle * 0.77288) + 80.389));
            shooterSubsystem.pointTurretPosition(((adjustedAngle * 0.77288) + 80.389));
            
        }
        else{
            // if(stopTurret == false) {
            //     shooterSubsystem.pointTurret();
            // } else {
            //     shooterSubsystem.pointTurret();
            // }
        }
        //System.out.println("Encoder: " + shooterSubsystem.getEncoder());
        if(controller.getStartButtonPressed()) {
            shooterSubsystem.resetOffset();
        }

        if(controller.getBackButtonPressed()) {
            shooterSubsystem.resetVelocityOffset();
        }

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

        if(down.get()) {
            shooterSubsystem.setTrajectoryAngle(0.125);
        }
        else if(up.get()) {
            shooterSubsystem.setTrajectoryAngle(-0.125);
        }
        else if(stopTurret == false) {
            shooterSubsystem.motionProfileAngleCalculated();
        } 
        else {
            shooterSubsystem.setTrajectoryAngle(0);
        }

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
