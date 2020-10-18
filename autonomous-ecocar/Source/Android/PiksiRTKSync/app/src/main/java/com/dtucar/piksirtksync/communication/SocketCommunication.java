package com.dtucar.piksirtksync.communication;

import android.app.Activity;
import android.os.AsyncTask;
import android.util.Log;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.net.ConnectException;
import java.net.HttpURLConnection;
import java.net.Socket;
import java.net.URL;
import java.net.UnknownHostException;

/**
 * Created by Henning on 19/02/2017.
 */

public class SocketCommunication extends DataCommunicationInterface {
	private static final String TAG = "SocketCommunication";
	private String response;
	private boolean isConnected;
	private SocketThread socketThread;
	private String serverAddress;

	public SocketCommunication(Activity activity) {
		super(activity);
		connect();
	}

	public void connect() {
		Log.d(TAG, "connect");
		new GetWebString(this).execute("https://dtucar.com/a/ip_sync.php?ident=665");
	}

	protected void startConnection(String serverAddress) throws CommunicationException {
		if(serverAddress == null) {
			Log.e(TAG, "ServerAddress was null");
			// TODO Do something about it
			throw new CommunicationException("ServerAddress was null");
		}
		this.serverAddress = serverAddress.trim();
		socketThread = new SocketThread(this, this.serverAddress);
		Log.d(TAG, "Got server IP: " + this.serverAddress);
		socketThread.start();
	}

	@Override
	public void send(byte[] data) throws CommunicationException {
		if(socketThread == null || !socketThread.isRunning()) {
			socketThread = new SocketThread(this, serverAddress);
			socketThread.start();
		}
		socketThread.sendMessage(data);
		// TODO Throw exception if not sent
	}

	@Override
	public boolean isConnected() {
		if(socketThread != null) {
			return socketThread.isRunning() && socketThread.isConnected();
		} else {
			Log.w(TAG, "SocketThread was null in isConnected");
			return false;
		}
	}

	public void disconnect() {
		Log.d(TAG, "disconnect");
		if(socketThread != null) socketThread.kill();
	}

	private class GetWebString extends AsyncTask<String, Void, String> {
		private SocketCommunication socketCommunication;

		public GetWebString(SocketCommunication socketCommunication) {
			this.socketCommunication = socketCommunication;
		}

		protected String doInBackground(String... urls) {
			HttpURLConnection urlConnection = null;
			BufferedReader reader = null;
			String response = null;
			try {
				URL url = new URL(urls[0]);
				urlConnection = (HttpURLConnection) url.openConnection();
				urlConnection.setRequestMethod("GET");
				urlConnection.connect();
				InputStream inputStream = urlConnection.getInputStream();
				StringBuffer buffer = new StringBuffer();
				if(inputStream == null) {
					return null;
				}
				reader = new BufferedReader(new InputStreamReader(inputStream));

				String line;
				while((line = reader.readLine()) != null) {
					buffer.append(line + "\n");
				}
				if(buffer.length() == 0) {
					return null;
				}
				response = buffer.toString();
			} catch(IOException e) {
				Log.e("PlaceholderFragment", "Error ", e);
				return null;
			} finally {
				if(urlConnection != null) {
					urlConnection.disconnect();
				}
				if(reader != null) {
					try {
						reader.close();
					} catch(final IOException e) {
						Log.e("PlaceholderFragment", "Error closing stream", e);
					}
				}
			}
			return response;
		}

		protected void onPostExecute(String serverAddress) {
			try {
				socketCommunication.startConnection(serverAddress);
			} catch(CommunicationException e) {
				e.printStackTrace();
			}
		}
	}
}