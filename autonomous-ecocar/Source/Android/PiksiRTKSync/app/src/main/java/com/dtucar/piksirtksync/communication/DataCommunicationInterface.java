package com.dtucar.piksirtksync.communication;

import android.app.Activity;

import java.util.ArrayList;

/**
 * Created by Henning on 10/06/2016.
 */
public abstract class DataCommunicationInterface {
	protected Activity activity;

	public DataCommunicationInterface(Activity activity) {
		this.activity = activity;
	}

	private ArrayList<OnDataReceivedListener> onDataReceivedListeners = new ArrayList<>();

	public abstract void send(byte[] data) throws CommunicationException;
	public abstract boolean isConnected();

	protected void onDataReceived(byte[] data) {
		for(int i = 0; i < onDataReceivedListeners.size(); i++) {
			onDataReceivedListeners.get(i).onDataReceived(data);
		}
	}

	public void registerMessageReceivedListener(OnDataReceivedListener listener) {
		onDataReceivedListeners.add(listener);
	}

	public void unregisterMessageReceivedListener(OnDataReceivedListener listener) {
		onDataReceivedListeners.remove(listener);
	}

	public enum CommunicationType {
		BLUETOOTH, USB, SOCKET
	}
}
