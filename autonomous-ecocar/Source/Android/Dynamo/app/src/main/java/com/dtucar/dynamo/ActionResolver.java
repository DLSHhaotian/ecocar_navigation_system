package com.dtucar.dynamo;

import android.app.Activity;
import android.util.Log;

import com.dtucar.dynamo.communication.CommunicationException;
import com.dtucar.dynamo.communication.CommunicationInterface;
import com.dtucar.dynamo.communication.OnMessageReceivedListener;
import com.dtucar.dynamo.communication.SocketCommunication;

/**
 * Created by Henning on 19/02/2017.
 */
public class ActionResolver {
    private static final String TAG = "ActionResolver";
    private static ActionResolver instance = new ActionResolver();
    private CommunicationInterface communication;

    public static ActionResolver getInstance() {
        return instance;
    }

    private ActionResolver() {
    }

    public void initialize(Activity activity) {
        communication = new SocketCommunication();
    }

    public void sendMessage(String message) throws CommunicationException {
        if(communication == null) {
            Log.w(TAG, "SocketCommunication was null. Reconstructed.");
            communication = new SocketCommunication();
        }
        communication.send(message);
    }

    public boolean isConnected() {
		if(communication == null) {
			Log.w(TAG, "SocketCommunication was null. Reconstructed.");
			communication = new SocketCommunication();
		}
		return communication.isConnected();
	}

    public void disconnect() {
        if(communication != null) communication.disconnect();
    }

    public void registerMessageReceivedListener(OnMessageReceivedListener listener) {
        if(communication == null) {
            Log.w(TAG, "SocketCommunication was null. Reconstructed.");
            communication = new SocketCommunication();
        }
        communication.registerMessageReceivedListener(listener);
    }

    public void unregisterMessageReceivedListener(OnMessageReceivedListener listener) {
        if(communication != null) {
            communication.unregisterMessageReceivedListener(listener);
        }
    }

	public void reconnect() {
		communication.disconnect();
		communication.connect();
	}
}
