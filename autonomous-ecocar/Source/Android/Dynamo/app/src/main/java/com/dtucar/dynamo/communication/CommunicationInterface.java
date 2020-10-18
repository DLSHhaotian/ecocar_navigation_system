package com.dtucar.dynamo.communication;

import android.app.Activity;

import java.util.ArrayList;

/**
 * Created by Henning on 10/06/2016.
 */
public abstract class CommunicationInterface {
    public CommunicationInterface() {

    }

    private ArrayList<OnMessageReceivedListener> onMessageReceivedListeners = new ArrayList<>();

    public abstract void send(String message) throws CommunicationException;

    public abstract boolean isConnected();

    public abstract void disconnect();

	public abstract void connect();

    protected void onMessageReceived(String message) {
        for(int i = 0; i < onMessageReceivedListeners.size(); i++) {
            onMessageReceivedListeners.get(i).onMessageReceived(message);
        }
    }

    public void registerMessageReceivedListener(OnMessageReceivedListener listener) {
        onMessageReceivedListeners.add(listener);
    }

    public void unregisterMessageReceivedListener(OnMessageReceivedListener listener) {
        onMessageReceivedListeners.remove(listener);
    }

    public enum CommunicationType {
        BLUETOOTH, USB, SOCKET
    }
}
