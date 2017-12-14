import java.io.BufferedReader;
import java.io.IOException;
import java.net.Socket;

public class SocketReader implements Runnable {
	BufferedReader s;
	Socket c;

	public SocketReader(BufferedReader s, Socket c) {
		this.s = s;
		this.c = c;
	}

	public void run() {
		while (true) {
			if (!Connect.stopReading) {
				try {
					if (s.ready()) {
						// System.out.println(s.readLine());
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}