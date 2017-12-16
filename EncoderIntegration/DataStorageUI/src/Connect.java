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
	
		file = new File("c://Users//NeilHazra//Dropbox//MachineLearningData//DistancePowerTest//DistancePowerTest_Power" +  Double.toString(power) + ".txt");
		file.createNewFile();
		FileWriter writeFile = new FileWriter(file);
		
		System.out.println("Waiting For RPI to connect");
		ServerSocket sock = new ServerSocket(3800);
		Socket client = sock.accept();
		PrintWriter writer =  new PrintWriter(client.getOutputStream(), true);
		BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream()));
		
		writeFile.write("Time," + "Power," + "Voltage," + "x," + "y," + "RightEncoder," + "LeftEncoder," + System.lineSeparator());
		writeFile.flush();
		int i = 1;
		while (true) {
			double distance = 0;
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
			writer.println(Double.toString(power*2-1));
			long initialTime = System.currentTimeMillis();
			writer.flush();
			
			while (System.currentTimeMillis() - initialTime < time) {
				;
			}
			while(!in.ready()) ;
			String s = in.readLine();
			System.out.println(s); //Replace this with more code
			String[] data = s.split(";");
			double rightEncoder = Double.parseDouble(data[0]);
			double leftEncoder = Double.parseDouble(data[1]);
			double voltage = Double.parseDouble(data[2]);
			
			System.out.println("Trial Ended, Enter x-distance (m)");
			double x = scanner.nextDouble();
			System.out.println("Enter y-distance (m)");
			double y = scanner.nextDouble();
			line = time/1000.0 + "," + power + "," + voltage + "," + x + "," + y + "," + rightEncoder + "," + leftEncoder + ","  + System.lineSeparator();
			writeFile.write(line);			
			writeFile.flush();
			i++;
		}
		System.out.println("Closing Connection and File");
		sock.close();
		scanner.close();
		writeFile.close();
	}
}