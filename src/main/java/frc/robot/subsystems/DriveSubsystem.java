// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Framework for subsystem and command files to work.
package frc.robot.subsystems; 

//SparkMax Controller Imports
import com.revrobotics.CANSparkMax; //Using CAN to communicate from the robo-rio to the SparkMax controllers
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //Select between brushless & non-brushless motors
import com.revrobotics.RelativeEncoder; //Built-in encoder on the motor 
import com.revrobotics.CANSparkMax.IdleMode; //Toggling Brake/Coast mode

//Meacanum Drive Imports
import edu.wpi.first.wpilibj.drive.MecanumDrive; //Drives the robot with driveCartesian(), driveCartesianIK(), or drivePolar() 
import edu.wpi.first.math.kinematics.MecanumDriveKinematics; //Makes conversion from ChassisVelocity to WheelSpeeds 
import edu.wpi.first.math.kinematics.MecanumDriveOdometry; //Allows the tracking of the robot's position on a field
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions; // Holds each of the four wheel positions in EncoderCounts
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds; // Holds each of the four wheel speeds in RPM

//Pose Imports
import edu.wpi.first.math.geometry.Pose2d; //Represents a 2D pose containing translational and rotational elements
import edu.wpi.first.math.geometry.Rotation2d; //Rotates the robot in the 2D Space
import edu.wpi.first.math.geometry.Translation2d; //Translates the robot in the 2D Space

//Nav-X Gyro Imports
import edu.wpi.first.wpilibj.SPI; //Represents an SPI bus port. 
import com.kauailabs.navx.frc.AHRS; //Allows the use of the GYRO

//Path Planner (PP) Imports
import com.pathplanner.lib.PathPlanner; //Connects the PathPlanner program with the project
import com.pathplanner.lib.PathConstraints; //Sets the max velocity and max acceleration
import com.pathplanner.lib.PathPlannerTrajectory; //Holds and accsess a path from a .path file
import com.pathplanner.lib.commands.PPMecanumControllerCommand; //Converts the instructions in the .path to induvidual motor speeds
import java.util.ArrayList; //Each path is stored in an index in an arraylist 
import edu.wpi.first.math.controller.PIDController; //https://bit.ly/42HpgyE Drives the robot with P, I, and, D values

//WPILIB Command Imports
import edu.wpi.first.wpilibj2.command.CommandBase; //Creates the command framework
import edu.wpi.first.wpilibj2.command.SubsystemBase; //Creates the subsystem framework
import edu.wpi.first.wpilibj2.command.Command; //Normal command with start and end condition
import edu.wpi.first.wpilibj2.command.InstantCommand; //Command that runs and finishes instantly 
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup; //Group of commands that run sequentialy

//Constant Imports
import frc.robot.Constants.CanIDConstants; //Imports the Id's for the motor controllers
import frc.robot.Constants.DriveConstants; //Imports the WheelSpeed conversion formula

//Smart Dashboard Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Creates a GUI that shows telemetry of the robot

/** Creates a new MecanumDrive. */
public class DriveSubsystem extends SubsystemBase {

  //MotorController initilize. We are using 4 Rev Neo Motors, with 4 Rev Spark.Max motor controllers. One for each wheel
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
  Translation2d m_frontLeftLocation = new Translation2d(0.2773, 0.2773); //Must be changed!
  Translation2d m_frontRightLocation = new Translation2d(0.2773, -0.2773); //Must be changed!
  Translation2d m_backLeftLocation = new Translation2d(-0.2773, 0.2773); //Must be changed!
  Translation2d m_backRightLocation = new Translation2d(-0.2773, -0.2773); //Must be changed!

  //Creating a kinematics object using the wheel locations
  MecanumDriveKinematics kinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  
  //Defining the drivetrain type
  MecanumDrive mDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

  //Default field orientated drive to false
  static Boolean foDrive = false;

  //Gyro initlizing and configures the NavX to use SPI
  public static AHRS ahrs = new AHRS(SPI.Port.kMXP); ;

  //Stores the path with the "test.path" and the set path constraints 

  public PathPlannerTrajectory traj = PathPlanner.loadPath("test", new PathConstraints(0.2, .1));

  //Default method, Called when created.  
  public DriveSubsystem() {
    //Calls the configureMotors() method
    configureMotors();
  } 
    
  //Configure the motor controllers
  public void configureMotors(){
    //Invert motors because gearbox things 
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    //Set Max Current for each motor
    rearRight.setSmartCurrentLimit( 80);
    rearLeft.setSmartCurrentLimit(80);
    frontRight.setSmartCurrentLimit(80);
    frontLeft.setSmartCurrentLimit(80);

    //Sets the IdleMode to brake 
    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    rearLeft.setIdleMode(IdleMode.kBrake);
    rearRight.setIdleMode(IdleMode.kBrake);
  }  
  
  //Robot-Orientated Drive
  public void Drive(double xSpeed, double ySpeed, double zRotation) {
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  //Field-Orientated Drive
  public void Drive(double xSpeed, double ySpeed, double zRotation, Rotation2d gyroAngle) { 
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation, gyroAngle);
  }

  //PathPlanner Drive
  public void drivePP(MecanumDriveWheelSpeeds speeds) {
    frontLeft.set(speeds.frontLeftMetersPerSecond);
    frontRight.set(speeds.frontRightMetersPerSecond);
    rearRight.set(speeds.rearRightMetersPerSecond);
    rearLeft.set(speeds.rearLeftMetersPerSecond);
  }  
  
  //Path Following command
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
          this.resetOdometry(traj.getInitialHolonomicPose());
        }
        idleMode(IdleMode.kBrake);
      }),
      new PPMecanumControllerCommand(
        traj, 
        this::getPose, // Pose supplier
        this.kinematics, // MecanumDriveKinematics
        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        0.2, // Max wheel velocity meters per second
        this::drivePP, // MecanumDriveWheelSpeeds consumer
        true, // Shozld the path be automatically mirrored depending on alliance color. Optional, defaults to true
        this   // Requires this drive subsystem
      )
    );
  }

  //Flip foDrive status
  public CommandBase toggleFO(){
    return runOnce(() -> {
      foDrive = !foDrive;
    });
  }

  //Returns the status of foDrive 
  public static boolean getFODrive(){
    return foDrive;
  }
  
  //Creating a odemtry object using the wheel position encoder 
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
    kinematics,
    ahrs.getRotation2d(),
    new MecanumDriveWheelPositions(
      0, 0, 0, 0  
    /*m_frontLeftEncoder.getPosition(), m_frontRightEncoder.getPosition(),
      m_rearLeftEncoder.getPosition(), m_rearRightEncoder.getPosition()*/
    ),
    new Pose2d(1, 4, new Rotation2d())
  );

  //Resets the robots odemetry/pose
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        ahrs.getRotation2d(), new MecanumDriveWheelPositions(m_frontLeftEncoder.getPosition(), m_frontRightEncoder.getPosition(),
        m_rearLeftEncoder.getPosition(), m_rearRightEncoder.getPosition()), pose);
  }

  //Updates the pose of the robot
  public void updatePose() {
    var wheelPositions = new MecanumDriveWheelPositions(
    DriveConstants.kLinearDistanceConverter * m_frontLeftEncoder.getPosition(), DriveConstants.kLinearDistanceConverter * m_frontRightEncoder.getPosition(),
    DriveConstants.kLinearDistanceConverter * m_rearLeftEncoder.getPosition(), DriveConstants.kLinearDistanceConverter * m_rearRightEncoder.getPosition());

    var gyroAngle = ahrs.getRotation2d();

    m_odometry.update(gyroAngle, wheelPositions);
  }

  //returns the pose of the robot in meters
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  
  //Set the motor idle mode
  public void idleMode(IdleMode idleMode){
    frontLeft.setIdleMode(idleMode);
    frontRight.setIdleMode(idleMode);
    rearLeft.setIdleMode(idleMode);
    rearRight.setIdleMode(idleMode);
  }

  //Reset encoders to zero
  public void resetEncoders() {
    m_frontLeftEncoder.setPosition(0);
    m_frontRightEncoder.setPosition(0);
    m_rearLeftEncoder.setPosition(0);
    m_rearRightEncoder.setPosition(0);
  }


  //Returns the wheel speeds
  public MecanumDriveWheelSpeeds getWheelSpeeds () {
    return new MecanumDriveWheelSpeeds(
      DriveConstants.kLinearDistanceConverter * m_frontLeftEncoder.getVelocity(),
      DriveConstants.kLinearDistanceConverter * m_frontRightEncoder.getVelocity(),
      DriveConstants.kLinearDistanceConverter * m_rearLeftEncoder.getVelocity(),
      DriveConstants.kLinearDistanceConverter * m_rearRightEncoder.getVelocity());
  }

  //Displays the values in the DriverStation dashboard
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("frontLeftCount", m_frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("frontRightCount", m_frontRightEncoder.getPosition());
    SmartDashboard.putNumber("rearLeftCount", m_rearLeftEncoder.getPosition());
    SmartDashboard.putNumber("rearRightCount", m_rearRightEncoder.getPosition());

    SmartDashboard.putNumber("frontLeftSpeed", m_frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("frontRightSpeed", m_frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("rearRightSpeed", m_rearLeftEncoder.getVelocity());
    SmartDashboard.putNumber("rearLeftSpeed", m_rearRightEncoder.getVelocity());
  }

  //This method runs every 20ms 
  @Override
  public void periodic() {
    //Updating the pose of the robot every 20ms 
    updatePose();
    updateSmartDashboard();
  } 
}
