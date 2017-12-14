import java.io.PrintWriter;
import java.net.Socket;

public class SocketWriter implements Runnable {
	PrintWriter s;
	Socket c;

	public SocketWriter(PrintWriter s, Socket c) {
		this.s = s;
		this.c = c;
	}

	public void run() {
		while (true) {
			if (!Connect.stopWriting) {
				String st = Float.toString(Connect.getLeftPower()) + ";" + Float.toString(Connect.getRightPower()) + ";\n";
				s.print(st);
				s.flush();
				System.out.println(st);
			}
		}
	}
}
