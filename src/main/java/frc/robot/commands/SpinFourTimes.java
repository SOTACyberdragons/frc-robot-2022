// package frc.robot.commands;

// import frc.robot.Robot;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.command.Command;


// /**
//  *
//  */
// public class SpinFourTimes extends Command {
//     int counter = 0;
//     String currentColor;
//     double spinnerSpeed = 1; //range from -1 to 1
//     Timer timer; 
//     String previousColor; 
//     public SpinFourTimes() {
//         // Use requires() here to declare subsystem dependencies
//         // eg. requires(chassis);
//         requires(Robot.spinner);
  
//     }

//     // Called just before this Command runs the first time
//     protected void initialize() {
//         currentColor = Robot.spinner.getColor();
//         timer.start();
        
//         previousColor=currentColor; 


//     } 

//     // Called repeatedly when this Command is scheduled to run
//     protected void execute() {
//         currentColor = Robot.spinner.getColor();
//         if(currentColor != previousColor) {
//             counter++;
//             previousColor = currentColor;
//         }

//         // if(timer.get() <= 5) {
//         //     Robot.spinner.spinSpinner(spinnerSpeed);
//         // } else if(timer.get() > 5) {
//         //     Robot.spinner.stopSpinner();
//         // } else {
//         //     Robot.spinner.stopSpinner();
//         // }	
//     }

//     // Make this return true when this Command no longer needs to run execute()
//     protected boolean isFinished() {
//         return counter >= 32;
//         // if(counter<=32) {
//         //     return true;
//         // }
//         //     else{
//         //         return false;
//         //     }
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