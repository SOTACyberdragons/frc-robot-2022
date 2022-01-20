// package frc.robot.commands;

// import frc.robot.Robot;

// import edu.wpi.first.wpilibj.command.Command;


// /**
//  *
//  */
// public class SpinToColor extends Command {

//     String wantedColor; 
//     String currentColor;
//     String previousColor; 
//     double spinnerSpeed = 1; //range from -1 to 1
//     public SpinToColor(String color) {
//         // Use requires() here to declare subsystem dependencies
//         // eg. requires(chassis);
//         requires(Robot.spinner);
//         wantedColor = color;
//     }

//     // Called just before this Command runs the first time
//     protected void initialize() {
        

//     } 

//     // Called repeatedly when this Command is scheduled to run
//     protected void execute() {

//         currentColor = Robot.spinner.getColor();

//         previousColor = currentColor; 

//         if(currentColor != wantedColor) {
//             Robot.spinner.spinSpinner(spinnerSpeed);
//         } else if(currentColor == wantedColor) {
//             Robot.spinner.stopSpinner();
//         } else {
//             Robot.spinner.stopSpinner();
//         }	
//     }

//     // Make this return true when this Command no longer needs to run execute()
//     protected boolean isFinished() {
//         return false;
    
//     }

//     // Called once after isFinished returns true
//     protected void end() {
//         Robot.spinner.stopSpinner();
//     }

//     // Called when another command which requires one or more of the same
//     // subsystems is scheduled to run
//     protected void interrupted() {
//     }
// }