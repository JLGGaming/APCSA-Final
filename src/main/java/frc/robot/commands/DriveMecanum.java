// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Command framework imports
package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

//Importing methods from other files
import frc.robot.RobotContainer; //Get controller imports
import frc.robot.subsystems.DriveSubsystem; //Call the drive() method(s)

//SparkMax Imports 
import com.revrobotics.CANSparkMax.IdleMode; //Change the motors from brake to coast mode

//Gyro Imports
import edu.wpi.first.math.geometry.Rotation2d; // Gets the rotational heading of the robot


public class DriveMecanum extends CommandBase {
  /** Creates a new DriveMecanum. */

  //Constructor method, called once only
  public DriveMecanum() {
    //Make sure no command is running before calling this command,
    addRequirements(RobotContainer.m_Drivetrain);
    //This command will be paused, then resumed again once the m_Drivetrain command ends (prevents user input when following a path)
  }

  // alled when the command is initially scheduled. (Not used)
  @Override
  public void initialize() {
    //This will run once at every start of the command
  }

  //Called every time the scheduler runs while the command is scheduled. (20ms)
  @Override
  public void execute() {
    //Check if field orintated drive is active 
    boolean FieldOrintated = DriveSubsystem.getFODrive();

    //Get the XboxController inputs 
    double xSpeed = RobotContainer.driverController.getY(); //Up and down
    double ySpeed = -RobotContainer.driverController.getX(); //Left and right
    double zRotation = -RobotContainer.driverController.getZ(); //Twisting the joystick left and right

    //Get Gyro Angle (Robot heading)
    Rotation2d gyroAngle = DriveSubsystem.ahrs.getRotation2d(); //0-360 degrees

    //If field orintated drive is active, overload the Drive() method
    if (FieldOrintated) {
      RobotContainer.m_Drivetrain.idleMode(IdleMode.kCoast); //Motors set to coast mode
      RobotContainer.m_Drivetrain.Drive(xSpeed*0.2, ySpeed*0.2, zRotation*0.2, gyroAngle); //Drive with the gyro angle determining the front of the robot
    }
    else { //
      if (zRotation > 0.25 || zRotation < -0.25){ //Only take the Z value from the joystick if more than 25% input is present (the controller had slight drift)
        RobotContainer.m_Drivetrain.idleMode(IdleMode.kCoast); //Motors set to coast mode
        RobotContainer.m_Drivetrain.Drive(xSpeed*0.2, ySpeed*0.2, zRotation*0.2); //Drive with that rotation value
        // RobotContainer.m_Drivetrain.Drive(xSpeed*0.2, ySpeed*0.2, Math.abs(zRotation)/zRotation*Math.sqrt(Math.abs(zRotation)*0.2)); 
      }
      else{
        RobotContainer.m_Drivetrain.Drive(xSpeed*0.2, ySpeed*0.2, 0); //Drive without any rotation at all
        RobotContainer.m_Drivetrain.idleMode(IdleMode.kCoast); //Motors set to coast mode
      }
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
