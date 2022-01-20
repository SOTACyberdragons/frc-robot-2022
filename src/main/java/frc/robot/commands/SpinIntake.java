// package frc.robot.commands;

// import frc.robot.Robot;

// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;


// /**
//  *
//  */
// public class SpinIntake extends CommandBase {

//     double speed;
//     public SpinIntake(double speed) {
//         addRequirements(Robot.intake);
//         speed = this.speed;
//     }

//     // Called just before this Command runs the first time
//     @Override
//     public void initialize() {  

//     } 

//     // Called repeatedly when this Command is scheduled to run
//     @Override
//     public void execute() {
//         Robot.intake.setIntakeSpeed(speed);	

//     }

//     // Make this return true when this Command no longer needs to run execute()
//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     // Called once after isFinished returns true
//     @Override
//     public void end(boolean interrupted) {
//         Robot.intake.stopMoving();
//     }
// }