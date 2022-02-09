// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.math.trajectory.Trajectory;
// import frc.robot.ramsete.ExecuteTrajectory;

// public class DriveTest extends CommandBase {
//     Trajectory trajectory;

//     ExecuteTrajectory path;

//     public DriveTest(Trajectory trajectory)
//     {
//         this.trajectory = trajectory;
//     }

//     public void execute()
//     {
//         path = new ExecuteTrajectory(this.trajectory);

//         CommandScheduler.schedule()
//     }

//     public boolean isFinished()
//     {
//         return this.path.done;
//     }
// }