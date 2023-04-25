// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;


public class DriveMecanum extends CommandBase {
  /** Creates a new DriveMecanum. */
  
  AHRS ahrs;


  public DriveMecanum() {
    addRequirements(RobotContainer.m_Drivetrain);
    ahrs = new AHRS(SPI.Port.kMXP); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean FO = DriveSubsystem.getFODrive();
    double xSpeed = -RobotContainer.driverController.getLeftY();
    double ySpeed = -RobotContainer.driverController.getLeftX();
    double zRotation = -RobotContainer.driverController.getRightX();
    if (FO) {
      RobotContainer.m_Drivetrain.Drive(xSpeed, ySpeed, zRotation, ahrs.getRotation2d());
    }
    else {
      RobotContainer.m_Drivetrain.Drive(xSpeed, ySpeed, zRotation);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
