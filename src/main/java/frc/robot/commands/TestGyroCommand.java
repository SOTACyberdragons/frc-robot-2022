// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
// import frc.robot.subsystems.SpinWithGyro;

// public class TestGyroCommand extends CommandBase
// {
//     SpinWithGyro gyroSubsystem = new SpinWithGyro();

//     double turnLimit;
    
//     public void initialize() {
//         System.out.println("Gyro is initialized\n");

//         turnLimit = Robot.m_robotDrive.gyro.getAngle() + 90;
//     }
    
//     public void execute() {
//         gyroSubsystem.spin();
//     }
    
//     @Override
//     public boolean isFinished()
//     {
//         if (Robot.m_robotDrive.gyro.getAngle() >= turnLimit || 
//         Robot.m_robotDrive.gyro.getAngle() <= -turnLimit) 
//         {
//             return true;
//         }
        
//         return false;
//     }
// }