package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LoadTrajectory {

    public static RamseteCommand getLoadedTrajectory(String path)
    {
        Trajectory trajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
         }

         RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            RobotContainer.m_robotDrive::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                    Constants.ksVolts,
                    Constants.kvVoltSecondsPerMeter,
                    Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            RobotContainer.m_robotDrive::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            RobotContainer.m_robotDrive::tankDriveVolts,
            RobotContainer.m_robotDrive);

            ramseteCommand.andThen(ramseteCommand.andThen(() -> RobotContainer.m_robotDrive.tankDriveVolts(0, 0)));

            return ramseteCommand;
    }
}
