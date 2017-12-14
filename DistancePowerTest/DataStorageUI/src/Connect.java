import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Scanner;

public class Connect {
	public static Socket mySocket;
	public static PrintWriter out;
	public static BufferedReader in;
	public static String s = new String();
	public static float leftPower = 0;
	public static float rightPower = 0;

	public static boolean stopReading = true;
	public static boolean stopWriting = false;
	public static boolean trialDone = false;
	public static boolean end = false;
	public static double values[] = new double[2];
	public static File file;
	
	public static float getLeftPower()	{
		return leftPower;
	}
	public static float getRightPower()	{
		return rightPower;
	}
	public static void main(String[] args) throws IOException, InterruptedException {
		file = new File("c://Users//NeilHazra//Dropbox//MachineLearningData//DistancePowerTest//DistancePowerTest" +  (new SimpleDateFormat ("MM-dd--hh-mm")).format(new Date()) );
		file.createNewFile();
		Scanner scanner = new Scanner(System.in);
		FileWriter writeFile = new FileWriter(file);
		System.out.println("Waiting For RPI to connect");
		ServerSocket sock = new ServerSocket(3800);
		Socket client = sock.accept();
		PrintWriter writer =  new PrintWriter(client.getOutputStream(), true);
		//Thread t1 = new Thread(new SocketReader(new BufferedReader(new InputStreamReader(client.getInputStream())), client));
		//Thread t2 = new Thread(new SocketWriter(new PrintWriter(client.getOutputStream(), true), client));
		//t1.start();
		//t2.start();
		
		writeFile.write("Trial" + "," + "TargetDistance" + "," + "Time" + "," + "Power" + "," + "x" + "," + "y" + System.lineSeparator());
		writeFile.flush();
		int i = 1;
		while (true) {
			double distance = 0;
			double time = 0;
			float leftPowerTemp, rightPowerTemp;
			String line;
			System.out.println("Beginning Trial " + i + " Type false to end or true to continue");
			if(!scanner.nextBoolean())	{
				break;
			}
			System.out.println("Enter Target Distance(meters)");
			distance = scanner.nextDouble();
			System.out.println("Enter Run Time (seconds)");
			time = scanner.nextDouble();
			System.out.println("Enter Left Power (-1,1)");
			leftPowerTemp = scanner.nextFloat();
			System.out.println("Enter Right Power (-1,1)");
			rightPowerTemp = scanner.nextFloat();
			System.out.println(rightPowerTemp);
			System.out.println("Starting Run");
			time *= 1000; // convert to millis
			long initialTime = System.currentTimeMillis();
			
			writer.print(time + ";");
			writer.print(Float.toString(leftPowerTemp) + ";");
			writer.println(Float.toString(rightPowerTemp));
			writer.flush();
			
			while (System.currentTimeMillis() - initialTime < time) {
			
			}
			System.out.println("Trial Ended, Enter x-distance (m)");
			double x = scanner.nextDouble();
			System.out.println("Enter y-distance (m)");
			double y = scanner.nextDouble();
			line = i + "," + distance + "," + time/1000 + "," + leftPowerTemp + "," + x + "," + y + System.lineSeparator();
			writeFile.write(line);			
			writeFile.flush();
			stopReading = true;
			i++;
		}
		System.out.println("Closing Connection and File");
		sock.close();
		scanner.close();
		writeFile.close();
	}
}