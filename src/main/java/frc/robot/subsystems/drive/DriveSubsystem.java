// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.lang.reflect.Field;
import java.sql.Time;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.PivotConfig.PivotId;
import frc.robot.subsystems.drive.commands.Drive;
import frc.robot.subsystems.drive.commands.DriveTest;
import frc.robot.utilities.Logger;

/** Represents a swerve drive style drivetrain. */
public class DriveSubsystem extends SubsystemBase {
  public static final double kMaxSpeed = 6; // 3 meters per second  
  public static final double kMaxAngularSpeed = 2 * Math.PI; // 1/2 rotation per second  
  
  private AnalogInput pixy = new AnalogInput(5);

  public static final double x = 0.276225; // 10.875"
  public static final double y = 0.301625; // 11.875"

  // OK
  private final Translation2d frontLeftLocation = new Translation2d(x, y);
  private final Translation2d frontRightLocation = new Translation2d(x, -y);
  private final Translation2d backLeftLocation = new Translation2d(-x, y);
  private final Translation2d backRightLocation = new Translation2d(-x, -y);

  // OK
  private final SwerveModule frontLeft = new SwerveModule(PivotConfig.getConfig(PivotId.FL));
  private final SwerveModule frontRight = new SwerveModule(PivotConfig.getConfig(PivotId.FR));
  private final SwerveModule backLeft = new SwerveModule(PivotConfig.getConfig(PivotId.BL));
  private final SwerveModule backRight = new SwerveModule(PivotConfig.getConfig(PivotId.BR));
  NetworkTableEntry xEntry;
  NetworkTableEntry yEntry;
  NetworkTableEntry FLtargetSpeedEntry,FLspeedEntry,BLtargetSpeedEntry,BLspeedEntry,FRtargetSpeedEntry,FRspeedEntry,BRtargetSpeedEntry,BRspeedEntry;
  // OK
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  public Field2d field = new Field2d();

  private double offset;

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

  public DriveSubsystem() {
    // NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // NetworkTable table = inst.getTable("odometry");
    // NetworkTable table1 = inst.getTable("speed");

    // xEntry = table.getEntry("X");
    // yEntry = table.getEntry("Y");
    // FLtargetSpeedEntry = table1.getEntry("FLtarget");
    // FLspeedEntry = table1.getEntry("FLactual");
    // FRtargetSpeedEntry = table1.getEntry("FRtarget");
    // FRspeedEntry = table1.getEntry("FRactual");
    // BLtargetSpeedEntry = table1.getEntry("BLtarget");
    // BLspeedEntry = table1.getEntry("BLactual");
    // BRtargetSpeedEntry = table1.getEntry("BRtarget");
    // BRspeedEntry = table1.getEntry("BRactual");
    
    // Shuffleboard.getTab("Field").add(field);
    gyro.reset();
  }

  public double getPixyVoltage() {
    return pixy.getVoltage();
  }

  public void initDefaultCommand() {
    setDefaultCommand(new Drive(this, true));
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void setOffset(double value) {
		offset = value;
	}

	public double getOffset() {
		return offset;
	}

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(gyro.getRotation2d().getRadians() - Math.toRadians(getOffset())))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
    //System.out.println("speed " + frontLeft.getVelocity());
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // System.out.println(desiredStates[0]);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);
    frontLeft.setDesiredStateAuto(desiredStates[0]);
    frontRight.setDesiredStateAuto(desiredStates[1]);
    backLeft.setDesiredStateAuto(desiredStates[2]);
    backRight.setDesiredStateAuto(desiredStates[3]);
  }
  //not used
  public double getGyroAngle() {
    return gyro.getYaw();
  }
  public double getGyroAngleDegrees(){
    return gyro.getRotation2d().getDegrees();
  }

  public void pointWheels(double angle) {
    frontLeft.setAngleD(angle);
    frontRight.setAngleD(angle);
    backLeft.setAngleD(angle);
    backRight.setAngleD(angle);
  }

  public void xWheels() {
    frontLeft.setAngleD(45);
    frontRight.setAngleD(-45);
    backLeft.setAngleD(-45);
    backRight.setAngleD(45);
  }

  public void setSpeeds(double speed) {
    frontLeft.setSpeed(speed);
    frontRight.setSpeed(speed);
    backLeft.setSpeed(speed);
    backRight.setSpeed(speed);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public Pose2d getPose() {
    Logger.log("x: " + odometry.getPoseMeters().getX() + "y: " + odometry.getPoseMeters().getY() + "gyro: " + getGyroAngle());
    //Logger.log("Time: "+ System.currentTimeMillis() + " x position: " + odometry.getPoseMeters().getX() + " y position: " + odometry.getPoseMeters().getY());
    
    // Logger.log(String.format("FLvel: %.2f, FLangle: %.2f\nFRvel: %.2f, FRangle: %.2f\nBLvel: %.2f, BLangle: %.2f\nBRvel: %.2f, BRangle: %.2f", 
    // frontLeft.getState().speedMetersPerSecond,
    // frontLeft.getState().angle.getDegrees(),
    // frontRight.getState().speedMetersPerSecond,
    // frontRight.getState().angle.getDegrees(),
    // backLeft.getState().speedMetersPerSecond,
    // backLeft.getState().angle.getDegrees(),
    // backRight.getState().speedMetersPerSecond,
    // backRight.getState().angle.getDegrees()
    // ));
    return odometry.getPoseMeters();
  }

  public void resetEncoders() {
    Stream.of(frontLeft, frontRight, backLeft, backRight).forEach(SwerveModule::resetEncoder);
  }
  @Override
  public void periodic() {
    updateOdometry();
    // xEntry.setDouble(getPose().getX());
    // yEntry.setDouble(getPose().getY());
    // FLtargetSpeedEntry.setDouble(frontLeft.targetSpeedAuto);
    // FLspeedEntry.setDouble(frontLeft.getVelocity());

    // FRtargetSpeedEntry.setDouble(frontRight.targetSpeedAuto);
    // FRspeedEntry.setDouble(frontRight.getVelocity());

    // BLtargetSpeedEntry.setDouble(backLeft.targetSpeedAuto);
    // BLspeedEntry.setDouble(backLeft.getVelocity());

    // BRtargetSpeedEntry.setDouble(backRight.targetSpeedAuto);
    // BRspeedEntry.setDouble(backRight.getVelocity());
    
    
    // field.setRobotPose(odometry.getPoseMeters());
  }
}
