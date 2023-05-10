// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.util.ArrayList;

// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.DriveSubsystem;

// public class FollowPath extends CommandBase {
  
//   private JSONArray path;
//   private Timer timer;
//   private double speed;
//   private Rotation2d angle;
//   private double zRotation;

//   private boolean finished = false;
  
//   /** Creates a new FollowPath. */
//   public FollowPath(JSONArray a) {
//     path = a;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(new DriveSubsystem());
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     timer = new Timer();
//     timer.start();
//     runPath(0);
//   }

//   public void runPath(int i){
//     JSONObject object = (JSONObject) path.get(i);

//     while (timer.get() < (double) object.get("time")) {
//       speed = (double) object.get("velocity");
//       System.out.println("Speed: " + speed);

//       angle = (Rotation2d) object.get("rotation");
//       System.out.println("Rotation: " + angle);

//       zRotation = (double) object.get("holonomicAngularVelocity");
//       System.out.println("zRotation: " + zRotation);

//       System.out.println("");
      
//       RobotContainer.m_Drivetrain.Drive(speed, angle, zRotation);
//     }

//     if (i<path.size()) {
//       runPath(i+1);
//     }

//   }
  


//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     int i = 0;
//     while (i < path.size() && (double) (((JSONObject) path.get(i)).get("time")) < timer.get()) {
//       i++;
//     }

//     if (i < path.size()) {
//     JSONObject object = (JSONObject) path.get(i); 
//     speed = (double) object.get("velocity");
//     System.out.println("Speed: " + speed);

//     angle = (Rotation2d) object.get("rotation");
//     System.out.println("Rotation: " + angle);

//     zRotation = (double) object.get("holonomicAngularVelocity");
//     System.out.println("zRotation: " + zRotation);

//     System.out.println("");
    
//     RobotContainer.m_Drivetrain.Drive(speed, angle, zRotation);
    
//     } else {
//       finished = true;
//     }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     timer.stop();
//     timer.reset();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return finished;
//   }
// }

