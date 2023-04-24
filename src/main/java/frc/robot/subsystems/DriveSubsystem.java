// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotContainer;
import frc.robot.Constants.SparkCosntants;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new MecanumDrive. */
  CANSparkMax frontLeft = new CANSparkMax(SparkCosntants.kFrontLeftChannel, MotorType.kBrushless);
  CANSparkMax rearLeft = new CANSparkMax(SparkCosntants.kRearLeftChannel,  MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(SparkCosntants.kFrontRightChannel,  MotorType.kBrushless);
  CANSparkMax rearRight = new CANSparkMax(SparkCosntants.kRearRightChannel,  MotorType.kBrushless);

  RelativeEncoder frontleftEncoder = frontLeft.getEncoder();
  RelativeEncoder frontrightEncoder = frontRight.getEncoder();
  RelativeEncoder rearleftEncoder = rearLeft.getEncoder();
  RelativeEncoder rearRightEncoder = rearRight.getEncoder();

  MecanumDrive mDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
  
  // private final MecanumDriveOdometry m_odometry; 


  public DriveSubsystem() {
    invertMotors();

  }

  public void Drive(double xSpeed, double ySpeed, double zRotation) {
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  public void invertMotors(){
    frontRight.setInverted(true);
    rearRight.setInverted(true);
  }  



  @Override
  public void periodic() {

  }  
}
