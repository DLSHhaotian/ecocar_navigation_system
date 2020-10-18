package com.dtucar.piksirtksync;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.ProgressBar;
import android.widget.TextView;

import com.dtucar.piksirtksync.communication.CommunicationException;
import com.dtucar.piksirtksync.communication.OnDataReceivedListener;
import com.dtucar.piksirtksync.communication.SocketCommunication;
import com.dtucar.piksirtksync.communication.UsbDataCommunication;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.TimeZone;

import static com.dtucar.piksirtksync.Constants.ACTION_USB_PERMISSION;

public class MainActivity extends Activity implements OnDataReceivedListener {
	public static final String TAG = "MainActivity";
	private UsbDataCommunication usbCommunication;
	private SocketCommunication socketCommunication;
	private Handler handler;
	boolean hasNewData;
	boolean isServiceRunning;
	private byte[] syncData;
	private final Object syncDataLock = new Object();
	// Status Views
	private View parentLayout;
	private TextView baseStatusTextView;
	private ProgressBar baseProgressBar;
	private TextView baseDataTextView;
	private TextView serverStatusTextView;
	private ProgressBar serverProgressBar;
	private TextView serverDataTextView;
	private long lastBaseMessageReceivedTime;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);
		parentLayout = findViewById(R.id.parentLayout);
		baseDataTextView = (TextView) findViewById(R.id.baseDataTextView);
		baseProgressBar = (ProgressBar) findViewById(R.id.baseProgressBar);
		baseStatusTextView = (TextView) findViewById(R.id.baseStatusTextView);
		serverDataTextView = (TextView) findViewById(R.id.serverDataTextView);
		serverProgressBar = (ProgressBar) findViewById(R.id.serverProgressBar);
		serverStatusTextView = (TextView) findViewById(R.id.serverStatusTextView);
		usbCommunication = new UsbDataCommunication(this);
		usbCommunication.registerMessageReceivedListener(this);
		IntentFilter filter = new IntentFilter();
		filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
		filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
		socketCommunication = new SocketCommunication(this);
		hasNewData = true;
		startSyncService();
	}

	@SuppressLint("InlinedApi")
	@Override
	protected void onResume() {
		super.onResume();
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
	}

	@Override
	protected void onPause() {
		super.onPause();
		getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
	}

	private void startSyncService() {
		if(Constants.DEBUG) Log.d(TAG, "Starting sync service");
		// Launch main loop
		isServiceRunning = true;
		handler = new Handler(getMainLooper());
		handler.postDelayed(new Runnable() {
			@Override
			public void run() {
				// START OF MAIN LOOP //
				// Check Base Station connection status
				checkBaseConnection();
				// If not OK, try to connect
				// TODO
				// Sync with HTTP server
				if(hasNewData) {
					boolean success = false;
					if(socketCommunication != null && socketCommunication.isConnected()) {
						synchronized(syncDataLock) {
							Log.d(TAG, "Sending data to server");
							try {
								socketCommunication.send(syncData);
								success = true;
							} catch(CommunicationException e) {
								e.printStackTrace();
								success = false;
							}
							// Wait
							Log.d(TAG, "Waiting");
							try {
								syncDataLock.wait();
							} catch(InterruptedException e) {
								e.printStackTrace();
							}
						}
					}
					displayServerStatus(success);
				}
				// END OF MAIN LOOP //
				// Reschedule loop execution
				if(isServiceRunning) handler.postDelayed(this, Constants.UPDATE_PERIOD);
			}
		}, Constants.UPDATE_PERIOD);
	}

	private void displayServerStatus(final boolean success) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				if(!success) {
					if(serverStatusTextView != null) {
						serverStatusTextView.setText(R.string.server_connection_error);
						serverStatusTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.errorColor));
					}
					if(serverProgressBar != null) {
						serverProgressBar.setVisibility(View.INVISIBLE);
					}
					if(serverDataTextView != null) {
						serverDataTextView.setText(getString(R.string.error));
						serverDataTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.errorColor));
					}
				} else {
					@SuppressLint("SimpleDateFormat") SimpleDateFormat sdf = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
					sdf.setTimeZone(TimeZone.getDefault());
					String currentDateTime = sdf.format(new Date());
					if(serverStatusTextView != null) {
						serverStatusTextView.setText(R.string.server_connection_ok);
						serverStatusTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.drivingForeground));
					}
					if(serverProgressBar != null) {
						serverProgressBar.setVisibility(View.VISIBLE);
					}
					if(serverDataTextView != null) {
						serverDataTextView.setText(getString(R.string.data_uploaded) + currentDateTime + "\n");
						serverDataTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.drivingForeground));
					}
				}
			}
		});
	}

	@Override
	public void onWindowFocusChanged(boolean hasFocus) {
		super.onWindowFocusChanged(hasFocus);
		if(hasFocus) {
			parentLayout.setSystemUiVisibility(
					View.SYSTEM_UI_FLAG_LAYOUT_STABLE
							| View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
							| View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
							| View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
							| View.SYSTEM_UI_FLAG_FULLSCREEN
							| View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
		}
	}

	private boolean checkBaseConnection() {
		// Check the timestamp of the last received message
		if(System.currentTimeMillis() - lastBaseMessageReceivedTime < Constants.NO_BASE_MESSAGE_RECEIVED_GRACE_TIME) {
			if(baseStatusTextView != null) {
				baseStatusTextView.setText(R.string.base_connection_ok);
				baseStatusTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.drivingForeground));
			}
			if(baseProgressBar != null) {
				baseProgressBar.setVisibility(View.VISIBLE);
			}
			return true;
		} else {
			if(baseStatusTextView != null) {
				if(usbCommunication.isConnected()) {
					baseStatusTextView.setText(R.string.base_connection_wait);
					baseStatusTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.warningColor));
				}
				else {
					baseStatusTextView.setText(R.string.base_connection_error);
					baseStatusTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.errorColor));
				}
			}
			if(baseProgressBar != null) {
				baseProgressBar.setVisibility(View.INVISIBLE);
			}
			if(baseDataTextView != null) {
				if(usbCommunication.isConnected()) {
					baseDataTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.errorColor));
				} else {
					baseDataTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.warningColor));
				}
			}
			return false;
		}
	}

	@SuppressLint("SetTextI18n")
	@Override
	public void onDataReceived(byte[] data) {
		// USB message received
		if(Constants.DEBUG) Log.d(TAG, "Data received");
		hasNewData = true;
		lastBaseMessageReceivedTime = System.currentTimeMillis();
		synchronized(syncDataLock) {
			syncData = data;
			syncDataLock.notifyAll();
		}
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				@SuppressLint("SimpleDateFormat") SimpleDateFormat sdf = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
				sdf.setTimeZone(TimeZone.getDefault());
				String currentDateTime = sdf.format(new Date());
				// Update status screen
				if(baseDataTextView != null) {
					baseDataTextView.setText(getString(R.string.data_received) + currentDateTime + "\n");
					baseDataTextView.setTextColor(ContextCompat.getColor(MainActivity.this, R.color.drivingForeground));
				}
			}
		});
	}

	public void reconnectSocket() {
		socketCommunication.disconnect();
		socketCommunication.connect();
	}
}
