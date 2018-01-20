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

public class Main {
    public static Socket mySocket;
    public static String s = new String();
    public static float leftPower = 0;
    public static float rightPower = 0;
    public static File file;
    public static void main(String[] args) throws IOException, InterruptedException {
        Scanner scanner = new Scanner(System.in);
        System.out.println("Enter Voltage");
        double voltage = scanner.nextFloat();
        System.out.println("Enter File Name");
        String name = scanner.next();
        file = new File("D:\\Personal\\Neil\\Research\\WSSEF\\Output\\Voltage-" +  Double.toString(voltage) + " " + name + " " + (new SimpleDateFormat("MM-dd--hh-mm")).format(new Date()) + ".txt");
        file.createNewFile();
        FileWriter writeFile = new FileWriter(file);
        System.out.println("Waiting For RPI to connect");
        ServerSocket sock = new ServerSocket(3800);
        Socket client = sock.accept();
        PrintWriter writer =  new PrintWriter(client.getOutputStream(), true);
        BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream()));
        writeFile.write("Trial Number, Time, Power, Voltage, actualx, actualy, EncoderX, En/coderY, AccelDispX, AccelDispY, kalmanx, kalmany" + System.lineSeparator());
        writeFile.flush();
        int i = 1;
        while (true) {
            double time = 0;
            String line;
            System.out.println("Beginning Trial " + i + " Type false to end or true to continue");
            if(!scanner.nextBoolean())  {
                break;
            }
            System.out.println("Enter Run Time (seconds)");
            time = scanner.nextDouble()*1000;
            System.out.println("Starting Run");
            writer.print(time + ";");
            writer.print(Double.toString(voltage) + ";");
            writer.println(Double.toString((voltage)));
            long initialTime = System.currentTimeMillis();
            writer.flush();
            while (System.currentTimeMillis() - initialTime < time) {
                Thread.sleep(5);
            }
            while(!in.ready()) ;

            String s = in.readLine();
            System.out.println(s); //Replace this with more code
            String[] data = s.split(";");
            double batteryVoltage = Double.parseDouble(data[0]);
            double encoderX = Double.parseDouble(data[1]);
            double encoderY = Double.parseDouble(data[2]);
            double dispXA = Double.parseDouble(data[3]);
            double dispYA = Double.parseDouble(data[4]);
            double kalmanX = Double.parseDouble(data[5]);
            double kalmanY = Double.parseDouble(data[6]);

            System.out.println("Trial Ended, Enter offset (x-distance) (inch)");
            double x = scanner.nextDouble()*2.54;
            System.out.println("Enter Staright distance(y-distance) (inch)");
            double y = scanner.nextDouble()*2.54;
            line = i + "," + time/1000.0 + "," + batteryVoltage + "," + voltage + "," + x + "," + y + ","  + encoderX*100 + "," + encoderY*100 + "," + dispXA*100 + "," + dispYA*100 + "," + kalmanX*100 + "," + kalmanY*100 + System.lineSeparator();
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
