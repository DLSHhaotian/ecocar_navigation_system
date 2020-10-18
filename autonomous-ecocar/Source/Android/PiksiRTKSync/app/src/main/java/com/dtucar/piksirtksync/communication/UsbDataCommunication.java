package com.dtucar.piksirtksync.communication;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.util.Log;

import com.dtucar.piksirtksync.Constants;
import com.ftdi.j2xx.D2xxManager;
import com.ftdi.j2xx.FT_Device;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

import static com.dtucar.piksirtksync.Constants.ACTION_USB_PERMISSION;

/**
 * Created by Henning on 10/06/2016.
 */
public class UsbDataCommunication extends DataCommunicationInterface {
	private static final String TAG = "UsbDataCommunication";
	public static D2xxManager ftD2xx = null;
	private static final int BAUDRATE = 115200;
	private int DevCount = -1;
	private boolean isConnected;
	final byte XON = 0x11;     /* Resume transmission */
	final byte XOFF = 0x13;    /* Pause transmission */
	private FT_Device ftDev;
	private ReadThread readThread;

	public UsbDataCommunication(Activity activity) {
		super(activity);
		try {
			ftD2xx = D2xxManager.getInstance(activity);
		} catch(D2xxManager.D2xxException e) {
			Log.e("FTDI_HT", "getInstance fail!!");
		}
		UsbManager usbManager = (UsbManager) activity.getSystemService(Context.USB_SERVICE);
		HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
		Iterator<UsbDevice> deviceIterator = deviceList.values().iterator();
		PendingIntent pi = PendingIntent.getBroadcast(activity, 0, new Intent(ACTION_USB_PERMISSION), 0);
		readThread = new ReadThread();
		while(deviceIterator.hasNext()) {
			UsbDevice d = deviceIterator.next();
			Log.d(TAG, "Found device: " + String.format("%04X:%04X", d.getVendorId(), d.getProductId()));
			activity.registerReceiver(usbReceiver, new IntentFilter(ACTION_USB_PERMISSION));
			if(!usbManager.hasPermission(d)) {
				usbManager.requestPermission(d, pi);
			} else {
				connect();
			}
			break;
		}
	}

	private void createDeviceList() {
		int tempDevCount = ftD2xx.createDeviceInfoList(activity);
		if(tempDevCount > 0) {
			if(DevCount != tempDevCount) {
				DevCount = tempDevCount;
			}
		} else {
			DevCount = -1;
		}
	}

	public void connect() {
		createDeviceList();
		D2xxManager.FtDeviceInfoListNode infoListNode = ftD2xx.getDeviceInfoListDetail(0);
		Log.d(TAG, "Serial: " + infoListNode.serialNumber);
		if(ftD2xx != null && DevCount > 0) {
			Log.d(TAG, "Trying to connect");
			ftDev = ftD2xx.openByIndex(activity, 0);
			// TODO Check null
			ftDev.setBitMode((byte) 0, D2xxManager.FT_BITMODE_RESET);
			ftDev.setBaudRate(BAUDRATE);
			ftDev.setDataCharacteristics(D2xxManager.FT_DATA_BITS_8, D2xxManager.FT_STOP_BITS_1, D2xxManager.FT_PARITY_NONE);
			ftDev.setFlowControl(D2xxManager.FT_FLOW_RTS_CTS, XON, XOFF);
		} else {
			Log.e(TAG, "Connect error");
			return;
		}
		isConnected = true;
		readThread.start();
	}

	public void disconnect() {
		DevCount = -1;
		isConnected = false;
		try {
			Thread.sleep(50);
		} catch(InterruptedException e) {
			e.printStackTrace();
		}
		if(ftDev != null) {
			if(ftDev.isOpen()) {
				ftDev.close();
			}
		}
	}

	@Override
	public boolean isConnected() {
		return isConnected;
	}

	@Override
	public void send(byte[] data) {
		// Sending is currently not used
		/*if(ftDev != null && ftDev.isOpen()) {
			int numBytes = message.length();
			for(int i = 0; i < numBytes; i++) {
				writeBuffer[i] = (byte) (message.charAt(i));
			}
			ftDev.write(writeBuffer, numBytes);
		} else {
			Log.e(TAG, "USB not connected");
		}*/
	}

	BroadcastReceiver usbReceiver = new BroadcastReceiver() {
		public void onReceive(Context context, Intent intent) {
			String action = intent.getAction();
			if(UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
				if(Constants.DEBUG) {
					Log.d(TAG, "Device attached");
				}
				if(!isConnected()) {
					if(Constants.DEBUG) {
						Log.d(TAG, "Device attached begin");
					}
					connect();
				}
			} else if(UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
				if(Constants.DEBUG) {
					Log.d(TAG, "Device detached");
				}
				disconnect();
			} else if(ACTION_USB_PERMISSION.equals(action)) {
				if(Constants.DEBUG) {
					Log.d(TAG, "Request permission");
				}
				synchronized(this) {
					if(!isConnected()) {
						if(Constants.DEBUG) {
							Log.d(TAG, "Request permission begin");
						}
						connect();
					}
				}
			}
		}
	};

	private class ReadThread extends Thread {
		@Override
		public void run() {
			while(isConnected()) {
				try {
					Thread.sleep(50);
				} catch(InterruptedException e) {
					e.printStackTrace();
				}
				int queueStatus = ftDev.getQueueStatus();
				byte[] data = new byte[queueStatus + 1];
				if(queueStatus > 0) {
					ftDev.read(data, queueStatus);
					Log.d(TAG, "Data read");
					onDataReceived(data);
				}
			}
			Log.e(TAG, "ReadThread terminated");
		}
	}
}
