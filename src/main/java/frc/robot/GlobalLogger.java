package frc.robot;

import java.util.Vector;

public class GlobalLogger {
    private static Vector<String> logger = new Vector<String>();

    public static void addMessage(String inputString)
    {

        logger.add(inputString);

        if (logger.size() == 15) {
            display();
            
            System.exit(0);
        }
    }

    public static void display()
    {
        for (String s : logger) {
            System.out.println(s);
        }
    }
}
