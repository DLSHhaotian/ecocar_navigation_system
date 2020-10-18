package com.dtucar.dynamo.communication;

import android.util.Log;

import com.dtucar.dynamo.ActionResolver;
import com.dtucar.dynamo.Constants;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

/**
 * Created by Henning on 19/02/2017.
 */

public class SocketThread extends Thread {
    private static final String TAG = "SocketThread";
	private final String serverAddress;
	private CommunicationInterface communicationInterface;
    private boolean running;
    private String message;
    private final Object lock = new Object();
	private boolean connected;

	public SocketThread(CommunicationInterface communicationInterface, String serverAddress) {
        this.communicationInterface = communicationInterface;
		this.serverAddress = serverAddress;
		connected = false;
    }

    public boolean isRunning() {
        return running;
    }

    @Override
    public void run() {
        running = true;
        Socket socket = null;
        try {
			Log.d(TAG, "Opening Socket");
            socket = new Socket(serverAddress, Constants.SERVER_PORT);
            OutputStream out = socket.getOutputStream();
            PrintWriter writer = new PrintWriter(out);
            BufferedReader br = new BufferedReader(new InputStreamReader(socket.getInputStream()));
			Log.d(TAG, "Connected to Socket");
			connected = true;
            while(running) {
				if(message == null) continue;
                synchronized(lock) {
                    Log.d(TAG, "Sending message: " + message);
                    long time = System.currentTimeMillis();
                    // Send
                    writer.println(message);
                    writer.flush();
                    // Receive
                    Log.d(TAG, "Reading message");
					if(br.ready()) {
						String result = br.readLine();
						Log.d(TAG, "Time: " + (System.currentTimeMillis() - time));
						Log.d(TAG, "Received: " + result);
						communicationInterface.onMessageReceived(result);
					} else {
						Log.d(TAG, "Nothing received");
					}
                    // Wait
                    Log.d(TAG, "Waiting");
                    try {
                        lock.wait();
                    } catch(InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
            br.close();
            out.close();
            writer.close();
        } catch(UnknownHostException e) {
            Log.e(TAG, "UnknownHostException: " + e.toString());
			Log.d(TAG, "Retrying connection... (" + serverAddress + ")");
        } catch(IOException e) {
            Log.e(TAG, "IOException: " + e.toString());
			if(e.toString().contains("Connection refused")) {
				// Server was not ready yet. Retry.
			} else {
				e.printStackTrace();
			}
        } finally {
            if(socket != null) {
                try {
                    socket.close();
                } catch(IOException e) {
                    e.printStackTrace();
                }
            }
        }
        if(running) {
			// If running is still true, something crashed. Retry connection.
			try {
				sleep(5000);
			} catch(InterruptedException ie) {
				ie.printStackTrace();
			}
			Log.d(TAG, "Retrying connection...");
			run();
		}
    }

    public void sendMessage(String message) {
        synchronized(lock) {
            this.message = message;
            lock.notifyAll();
        }
    }

    public void kill() {
        running = false;
    }

	public boolean isConnected() {
		return connected;
	}
}