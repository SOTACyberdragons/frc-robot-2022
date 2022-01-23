package frc.robot;

import java.util.Date;

import frc.robot.subsystems.Drivetrain;

public class TestDriveFunctions implements Runnable {
    private static long totalDriveTime = 0;

    public int timesRan = 0;

    public TestDriveFunctions(long time)
    {
        totalDriveTime = time;
    }

    private void driveForXMilliseconds() throws InterruptedException
    {
        Date date = new Date();

        long initialTime = date.getTime();

        Drivetrain train = new Drivetrain();

        while (true) {
            if (date.getTime() - initialTime == totalDriveTime) {
                this.timesRan++;

                break;
            }

            train.drive(1, 1);

            Thread.sleep(1000);
        }
    }

    public void run() 
    {
        try {
            driveForXMilliseconds();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
