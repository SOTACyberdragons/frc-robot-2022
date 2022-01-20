// package frc.robot.commands;

// import frc.robot.Robot;
// import frc.robot.subsystems.Feeder;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class FeedBall extends CommandBase {

//     String direction = "";

//     private final Feeder m_feeder;
    
//     public FeedBall(Feeder feeder, String direction) {
//         m_feeder = feeder;
//         addRequirements(feeder);
//         this.direction = direction;
//     }

//     public FeedBall(Feeder feeder) {
//         m_feeder = feeder;
//         addRequirements(feeder);
//     }


//     // Called just before this Command runs the first time
//     @Override
//     public void initialize() {  

//     } 

//     // Called repeatedly when this Command is scheduled to run
//     @Override
//     public void execute() {
//         if(direction == "in") { 
//         Robot.feeder.feedIn();	
//         } else if(direction == "out") {
//             Robot.feeder.feedOut();
//         } else{
//             Robot.feeder.feedIn();
//         }
//         System.out.println("Feeding!!!!");
//     }

//     // Make this return true when this Command no longer needs to run execute()
//     @Override
//     public boolean isFinished() {
//         return false;
    
//     }

//     // Called once after isFinished returns true
//     @Override
//     public void end(boolean interrupted) {
//         Robot.feeder.stopFeeding();
//     }
    
// }