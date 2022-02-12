package frc.robot.ramsete;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class CustomRamsete extends CommandBase {
  private final Timer m_timer = new Timer();
  private final boolean m_usePID;
  public Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final RamseteController m_follower;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
  private final PIDController m_leftController;
  private final PIDController m_rightController;
  private final BiConsumer<Double, Double> m_output;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param wheelSpeeds A function that supplies the speeds of the left and right sides of the robot
   *     drive.
   * @param leftController The PIDController for the left side of the robot drive.
   * @param rightController The PIDController for the right side of the robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param requirements The subsystems to require.
   */
  public CustomRamsete(Trajectory trajectory) {
    m_trajectory = trajectory;

    m_pose = RobotContainer.m_robotDrive::getPose;

    m_follower = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
    m_feedforward = new SimpleMotorFeedforward(
      Constants.ksVolts,
      Constants.kvVoltSecondsPerMeter,
      Constants.kaVoltSecondsSquaredPerMeter);

    m_kinematics = Constants.kDriveKinematics;

    m_speeds = RobotContainer.m_robotDrive::getWheelSpeeds;

    m_leftController = new PIDController(Constants.kPDriveVel, 0, 0);
    m_rightController = new PIDController(Constants.kPDriveVel, 0, 0);
    m_output = RobotContainer.m_robotDrive::tankDriveVolts;

    m_usePID = true;

    addRequirements(RobotContainer.m_robotDrive);
  }
  

  @Override
  public void initialize() {
    m_prevTime = -1;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds =
        m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    if (m_usePID) {
      m_leftController.reset();
      m_rightController.reset();
    }
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    if (m_prevTime < 0) {
      m_output.accept(0.0, 0.0);
      m_prevTime = curTime;
      return;
    }

    var targetWheelSpeeds =
        m_kinematics.toWheelSpeeds(
            m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (m_usePID) {
      double leftFeedforward =
          m_feedforward.calculate(
              leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

      double rightFeedforward =
          m_feedforward.calculate(
              rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

      leftOutput =
          leftFeedforward
              + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

      rightOutput =
          rightFeedforward
              + m_rightController.calculate(
                  m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    m_output.accept(leftOutput, rightOutput);
    m_prevSpeeds = targetWheelSpeeds;
    m_prevTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();

    if (interrupted) {
      m_output.accept(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
