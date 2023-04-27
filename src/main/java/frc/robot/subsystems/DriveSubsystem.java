// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

//SparkMax Controller Imports
import com.revrobotics.CANSparkMax; //Using CAN to communicate
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //Brushless vs Non-Brushless
import com.revrobotics.RelativeEncoder; //Built-in encoder on the motor 

//Meacanum Drive Imports
import edu.wpi.first.math.kinematics.MecanumDriveKinematics; //Makes conversion from ChassisVelocity to WheelSpeeds 
import edu.wpi.first.math.kinematics.MecanumDriveOdometry; //Allows the tracking of the robot's position on a field
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions; // Holds the four wheel speed position
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds; // Holds the four wheel speed values 
// import edu.wpi.first.math.kinematics.ChassisSpeeds; //Represents the speed of the chassis 

//Pose Imports
import edu.wpi.first.math.geometry.Pose2d; //Represents a 2D pose containing translational and rotational elements
import edu.wpi.first.math.geometry.Rotation2d; //Rotates the robot in the 2D Space
import edu.wpi.first.math.geometry.Translation2d; //Translates the robot in the 2D Space

//Nav-X Gyro Imports
import edu.wpi.first.wpilibj.SPI; //Represents an SPI bus port. 
import com.kauailabs.navx.frc.AHRS; //Allows the use of the GYRO

//Mecanum Drive Imports 
import edu.wpi.first.wpilibj.drive.MecanumDrive; //Drives the robot with driveCartesian()

//WPILIB Command Imports
import edu.wpi.first.wpilibj2.command.CommandBase; //Creates the command framework
import edu.wpi.first.wpilibj2.command.SubsystemBase; //Creates the subsystem framework

//Constant Imports
import frc.robot.Constants.CanIDConstants; //Imports the Id's for the motor controllers


/** Creates a new MecanumDrive. */
public class DriveSubsystem extends SubsystemBase {

  //MotorController initilize. We are using 4 Rev Neo Motors, with 4 Rev SparkMax motor controllers. One for each wheel
  CANSparkMax frontLeft = new CANSparkMax(CanIDConstants.kFrontLeftChannel, MotorType.kBrushless);
  CANSparkMax rearLeft = new CANSparkMax(CanIDConstants.kRearLeftChannel,  MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(CanIDConstants.kFrontRightChannel,  MotorType.kBrushless);
  CANSparkMax rearRight = new CANSparkMax(CanIDConstants.kRearRightChannel,  MotorType.kBrushless);

  //Integrated Motor Encoders. One revolution of the motor is 42 encoder counts
  RelativeEncoder m_frontLeftEncoder = frontLeft.getEncoder();
  RelativeEncoder m_frontRightEncoder = frontRight.getEncoder();
  RelativeEncoder m_rearLeftEncoder = rearLeft.getEncoder();
  RelativeEncoder m_rearRightEncoder = rearRight.getEncoder();

  //2D Space where the wheels are relative to the center of the robot
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381); //Must be changed!
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381); //Must be changed!
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381); //Must be changed!
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381); //Must be changed!

  //Creating a kinematics object using the wheel locations
  MecanumDriveKinematics kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  
  //Defining the drivetrain type
  MecanumDrive mDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  //Default field orientated drive to false
  static Boolean foDrive = false;

  //Gyro initlizing
  public static AHRS ahrs;


  //Default method, Called when created.  
  public DriveSubsystem() {
    //Call the invertMotor() method
    invertMotors();
    //Configures the NavX to use SPI
    ahrs = new AHRS(SPI.Port.kMXP); 
  }
    
  //Invert motors because gearbox things 
  public void invertMotors(){
      frontRight.setInverted(true);
      rearRight.setInverted(true);
    }  
  


  //Robot-Orientated Drive
  public void Drive(double xSpeed, double ySpeed, double zRotation) {
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  //Field-Orientated Drive
  public void Drive(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) { 
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation, gyroAngle);
  }


  //Flip foDrive status
  public CommandBase toggleFO(){
    return runOnce(() -> {
      foDrive = !foDrive;
    });
  }

  //Creating a odemtry object using the wheel position encoder 
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
    kinematics,
    ahrs.getRotation2d(),
    new MecanumDriveWheelPositions(
      m_frontLeftEncoder.getPosition(), m_frontRightEncoder.getPosition(),
      m_rearLeftEncoder.getPosition(), m_rearRightEncoder.getPosition()
    ),
    new Pose2d(5.0, 13.5, new Rotation2d())
  );

  //Returns the status of foDrive 
  public static boolean getFODrive(){
    return foDrive;
  }

  //Updates the pose of the robot
  public void updatePose(){
    var wheelPositions = new MecanumDriveWheelPositions(
    m_frontLeftEncoder.getPosition(), m_frontRightEncoder.getPosition(),
    m_rearLeftEncoder.getPosition(), m_rearRightEncoder.getPosition());

    var gyroAngle = ahrs.getRotation2d();

    m_odometry.update(gyroAngle, wheelPositions);
  }

  //Resets the robots odemetry/pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        ahrs.getRotation2d(), new MecanumDriveWheelPositions(m_frontLeftEncoder.getPosition(), m_frontRightEncoder.getPosition(),
        m_rearLeftEncoder.getPosition(), m_rearRightEncoder.getPosition()), pose);
  }

  //returns the pose of the robot in meters
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }


  //Reset encoders to zero
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }
  

  //Returns the speed of each wheel 
  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(m_frontLeftEncoder.getCountsPerRevolution(), m_frontRightEncoder.getCountsPerRevolution(),
    m_rearLeftEncoder.getCountsPerRevolution(), m_rearRightEncoder.getCountsPerRevolution());
  }

 
  //This method runs every 20ms 
  @Override
  public void periodic() {
    //Updating the pose of the robot every 20ms 
    updatePose();
  }

 
}
