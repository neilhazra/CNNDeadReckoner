package com.company;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Scanner;
import javax.swing.SwingUtilities;

public class Main {
    public static Socket mySocket;
    public static String s = new String();
    public static float leftPower = 0;
    public static float rightPower = 0;
    public static File file;
    public static float getLeftPower()	{
        return leftPower;
    }
    public static float getRightPower()	{
        return rightPower;
    }
    public static void main(String[] args) throws IOException, InterruptedException {
        Scanner scanner = new Scanner(System.in);
        System.out.println("Enter Power (0,1)");
        double power = scanner.nextFloat();
        System.out.println("Enter File Name");
        String name = scanner.next();
        file = new File("C://Users//NeilHazra//Dropbox//Neil Research Project//MachineLearningData//DistancePowerTest//DistancePowerTest_Power//Power-" +  Double.toString(power) + " " + name + " " + (new SimpleDateFormat("MM-dd--hh-mm")).format(new Date()) + ".txt");
        file.createNewFile();
        FileWriter writeFile = new FileWriter(file);
        System.out.println("Waiting For RPI to connect");
        ServerSocket sock = new ServerSocket(3800);
        Socket client = sock.accept();
        PrintWriter writer =  new PrintWriter(client.getOutputStream(), true);
        BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream()));
        writeFile.write("Time, Power, Voltage, x, y, RightEncoderCount, LeftEncoderCount, EncoderX, EncoderY, Heading" + System.lineSeparator());
        writeFile.flush();
        int i = 1;
        while (true) {
            double time = 0;
            String line;
            System.out.println("Beginning Trial " + i + " Type false to end or true to continue");
            if(!scanner.nextBoolean())	{
                break;
            }
            System.out.println("Enter Run Time (seconds)");
            time = scanner.nextDouble()*1000;
            System.out.println("Starting Run");
            writer.print(time + ";");
            writer.print(Double.toString(power*2-1) + ";");
            writer.println(Double.toString((power*2-1)));
            long initialTime = System.currentTimeMillis();
            writer.flush();
            while (System.currentTimeMillis() - initialTime < time) {
                Thread.sleep(5);
            }
            while(!in.ready()) ;

            String s = in.readLine();
            System.out.println(s); //Replace this with more code
            String[] data = s.split(";");
            double rightEncoder = Double.parseDouble(data[0]);
            double leftEncoder = Double.parseDouble(data[1]);
            double voltage = Double.parseDouble(data[2]);
            double encoderX = Double.parseDouble(data[3]);
            double encoderY = Double.parseDouble(data[4]);
            double heading = Double.parseDouble(data[7]);
            System.out.println("Trial Ended, Enter x-distance (m)");
            double x = scanner.nextDouble();
            System.out.println("Enter y-distance (m)");
            double y = scanner.nextDouble();
            line = time/1000.0 + "," + power + "," + voltage + "," + x + "," + y + "," + rightEncoder + "," + leftEncoder + "," + encoderX + "," + encoderY + "," + heading + System.lineSeparator();
            writeFile.write(line);
            writeFile.flush();
            i++;
        }
        System.out.println("Closed Connection and File");
        sock.close();
        scanner.close();
        writeFile.close();
    }
}