package com.dtucar.dynamo;

import android.annotation.SuppressLint;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.webkit.WebView;
import android.widget.Button;
import android.widget.RelativeLayout;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.dtucar.dynamo.communication.CommunicationException;
import com.dtucar.dynamo.communication.OnMessageReceivedListener;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.concurrent.TimeUnit;

import io.github.controlwear.virtual.joystick.android.JoystickView;
import ng.max.slideview.SlideView;

public class ControlActivity extends MultiTouchActivity implements OnMapReadyCallback, OnMessageReceivedListener {
	private static final String TAG = "ControlActivity";

	private GoogleMap mMap;
	private Marker marker;
	private Handler handler;
	private LatLng position;
	private View parentLayout;
	private static final float ZOOM_LEVEL = 16f;
	private boolean burn, horn;
	private double steeringPercentage;
	private double brakePercentage;
	private double speed = 0, distanceWheel = 0;
	private int gear = 0, gpsStatus = 0;
	private long carTime;
	private long lastMessageReceivedTime = 0;

	int[][] states = new int[][] {
			new int[] { android.R.attr.state_enabled} // enabled
	};

	private Runnable syncLoop = new Runnable() {
		@Override
		public void run() {
			sendMessage();
			if(System.currentTimeMillis() - lastMessageReceivedTime > Constants.SYNC_TIMEOUT) {
				Log.d(TAG, "No connection");
//				if(ActionResolver.getInstance().isConnected()) {
//					Log.d(TAG, "Connection lost");
//					ActionResolver.getInstance().reconnect();
//				}	// TODO Reconnect (but only when the connection is not being established)
				runOnUiThread(new Runnable() {
					@Override
					public void run() {
						TextView speedView = (TextView) findViewById(R.id.speedView);
						speedView.setText("No connection");
					}
				});
			}
			handler.postDelayed(this, Constants.SYNC_LOOP_PERIOD);
		}
	};

	@SuppressLint("SetJavaScriptEnabled")
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_control);
		parentLayout = findViewById(R.id.parentLayout);
		handler = new Handler();
		JoystickView joystick = (JoystickView) findViewById(R.id.joystickView);
		final TextView steeringPercentageView = (TextView) findViewById(R.id.steeringPercentage);
		RelativeLayout.LayoutParams params = (RelativeLayout.LayoutParams) steeringPercentageView.getLayoutParams();
		steeringPercentageView.setWidth(joystick.getWidth());
		steeringPercentageView.setLayoutParams(params);
		joystick.setOnMoveListener(new JoystickView.OnMoveListener() {
			@SuppressLint({"SetTextI18n", "DefaultLocale"})
			@Override
			public void onMove(int angle, int strength) {
				double steering = Math.cos(Math.toRadians(angle)) * strength;
				steeringPercentage = steering;
				steeringPercentageView.setText(String.format("%.03f", steering) + "%");
			}
		});
		WebView streamWebView = (WebView) findViewById(R.id.streamWebView);
		streamWebView.getSettings().setLoadsImagesAutomatically(true);
		streamWebView.getSettings().setJavaScriptEnabled(true);
		streamWebView.setScrollBarStyle(View.SCROLLBARS_INSIDE_OVERLAY);
		streamWebView.loadUrl("https://dtucar.com/a/?r=" + Math.random());
		SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map);
		mapFragment.getMapAsync(this);
		final Button burnButton = (Button) findViewById(R.id.burnButton);
		final Animation holdAnimation = AnimationUtils.loadAnimation(this, R.anim.hold);
		burnButton.setOnTouchListener(new View.OnTouchListener() {
			@Override
			public boolean onTouch(View view, MotionEvent event) {
				switch(event.getAction()) {
					case MotionEvent.ACTION_DOWN:
						burn = true;
						view.startAnimation(holdAnimation);
						view.setBackgroundColor(Color.parseColor("#ff0000"));
						return false;
					case MotionEvent.ACTION_UP:
						burn = false;
						view.clearAnimation();
						view.setBackgroundResource(android.R.drawable.btn_default);
						return false;
				}
				return false;
			}
		});
		final Button hornButton = (Button) findViewById(R.id.hornButton);
		hornButton.setOnTouchListener(new View.OnTouchListener() {
			@Override
			public boolean onTouch(View view, MotionEvent event) {
				switch(event.getAction()) {
				case MotionEvent.ACTION_DOWN:
					horn = true;
					view.startAnimation(holdAnimation);
					view.setBackgroundColor(Color.parseColor("#4298f4"));
					return false;
				case MotionEvent.ACTION_UP:
					horn = false;
					view.clearAnimation();
					view.setBackgroundResource(android.R.drawable.btn_default);
					return false;
			}
				return false;
			}
		});
		TextView speedView = (TextView) findViewById(R.id.speedView);
		speedView.setText(speed + " " + getString(R.string.km_h) + "\n" + (gear == 0 ? "N" : gear) + " gear\n" + distanceWheel + "m");
		final SlideView slideView = ((SlideView) findViewById(R.id.slideView));
		final TextView brakePercentageView = (TextView) findViewById(R.id.brakePercentage);
		int defaultBrakeBackgroundColor;
		int defaultBrakeTargetBackgroundColor;
		if(android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.M) {
			defaultBrakeBackgroundColor = getColor(R.color.brakeBackground);
			defaultBrakeTargetBackgroundColor = getColor(R.color.brakeTargetBackground);
		} else {
			defaultBrakeBackgroundColor = 1043582207;
			defaultBrakeTargetBackgroundColor = 14494720;
		}
		final int brakeBackgroundColor = defaultBrakeBackgroundColor;
		final int brakeTargetBackgroundColor = defaultBrakeTargetBackgroundColor;
		slideView.getSlider().setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
			@SuppressLint("SetTextI18n")
			@Override
			public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
				slideView.onProgressChanged(seekBar, i, b);
				brakePercentage = seekBar.getProgress();
				brakePercentageView.setText(seekBar.getProgress() + "%");
				slideView.setSlideBackgroundColor(new ColorStateList(states, new int[] {Color.argb(
						(int)(Color.alpha(brakeBackgroundColor) + seekBar.getProgress() / 100f * (Color.alpha(brakeTargetBackgroundColor) - Color.alpha(brakeBackgroundColor))),
						(int)(Color.red(brakeBackgroundColor) + seekBar.getProgress() / 100f * (Color.red(brakeTargetBackgroundColor) - Color.red(brakeBackgroundColor))),
						(int)(Color.green(brakeBackgroundColor) + seekBar.getProgress() / 100f * (Color.green(brakeTargetBackgroundColor) - Color.green(brakeBackgroundColor))),
						(int)(Color.blue(brakeBackgroundColor) + seekBar.getProgress() / 100f * (Color.blue(brakeTargetBackgroundColor) - Color.blue(brakeBackgroundColor))) )}));
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar) {
			}

			@Override
			public void onStopTrackingTouch(SeekBar seekBar) {
			}
		});
	}

	@Override
	protected void onResume() {
		super.onResume();
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		getWindow().getDecorView().setSystemUiVisibility(
				View.SYSTEM_UI_FLAG_LAYOUT_STABLE
						| View.SYSTEM_UI_FLAG_LOW_PROFILE
						| View.SYSTEM_UI_FLAG_FULLSCREEN
						| View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
						| View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
						| View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
		handler.postDelayed(syncLoop, Constants.SYNC_LOOP_PERIOD);
	}

	/**
	 * This ensures that the immersive mode is enabled even after a resume of the activity.
	 * {@inheritDoc}
	 */
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

	@Override
	protected void onPause() {
		super.onPause();
		ActionResolver.getInstance().disconnect();
		getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		handler.removeCallbacks(syncLoop);
	}

	@Override
	protected void onStop() {
		super.onStop();
	}

	@Override
	public void onMapReady(GoogleMap googleMap) {
		mMap = googleMap;
		position = new LatLng(55.7834892, 12.5226897);
		MarkerOptions markerOptions = new MarkerOptions().position(position).title("Dynamo");
		marker = mMap.addMarker(markerOptions);
		mMap.moveCamera(CameraUpdateFactory.newLatLng(position));
		mMap.moveCamera(CameraUpdateFactory.zoomTo(ZOOM_LEVEL));
		ActionResolver.getInstance().registerMessageReceivedListener(this);
	}

	public void sendMessage() {
		JSONObject jsonMessage = new JSONObject();
		try {
			jsonMessage.put("t", System.currentTimeMillis());
			jsonMessage.put("burn", burn);
			jsonMessage.put("horn", horn);
			jsonMessage.put("steeringP", steeringPercentage);
			jsonMessage.put("brakeP", brakePercentage);
		} catch (JSONException e) {
			e.printStackTrace();
		}
		if(ActionResolver.getInstance().isConnected()) {
			try {
				ActionResolver.getInstance().sendMessage(jsonMessage.toString());
			} catch(CommunicationException e) {
				Toast.makeText(this, "Could not send message", Toast.LENGTH_SHORT).show();
				e.printStackTrace();
			}
		} else {
			Log.d(TAG, "Could not send message (not connected)");
		}
	}

	@SuppressLint("SetTextI18n")
	@Override
	public void onMessageReceived(String message) {
		Log.d(TAG, "Message received: " + message);
		if(message == null) return;
		try {
			JSONObject reader = new JSONObject(message);
			if(reader.has("t"))	carTime = reader.getLong("t");
			if(reader.has("sW"))	speed = reader.getDouble("sW");
			if(reader.has("gr"))	gear = reader.getInt("gr");
			if(reader.has("dW"))	distanceWheel = reader.getInt("dW");
			if(reader.has("la") && reader.has("lo")) {
				double latitude = reader.getDouble("la");
				double longitude = reader.getDouble("lo");
				position = new LatLng(latitude, longitude);
			}
			//double altitude = reader.getDouble("al");
			if(reader.has("gs"))	gpsStatus = reader.getInt("gs");
		} catch(JSONException e) {
			e.printStackTrace();
		}
		// Take actions based on messages
		runOnUiThread(new Runnable() {
			@SuppressLint("DefaultLocale")
			@Override
			public void run() {
				TextView speedView = (TextView) findViewById(R.id.speedView);
				speedView.setText(speed + " " + getString(R.string.km_h) + "\nGear " + (gear == 0 ? "N" : gear) + "\n" + distanceWheel + "m");
				LatLngBounds latLngBounds = mMap.getProjection().getVisibleRegion().latLngBounds;
				if(!latLngBounds.contains(position)) {
					mMap.moveCamera(CameraUpdateFactory.newLatLng(position));
				}
				marker.setPosition(position);
				TextView statusView = (TextView) findViewById(R.id.statusView);
				statusView.setText(gpsStatusToString(gpsStatus) + "\t" + String.format("%02d min, %02d sec",
						TimeUnit.MILLISECONDS.toMinutes(carTime),
						TimeUnit.MILLISECONDS.toSeconds(carTime) -
						TimeUnit.MINUTES.toSeconds(TimeUnit.MILLISECONDS.toMinutes(carTime))
				));
				lastMessageReceivedTime = System.currentTimeMillis();
			}
		});
	}

	private String gpsStatusToString(int gpsStatus) {
		switch(gpsStatus) {
			case 1:
				return "SPP";
			case 2:
				return "DGNSS";
			case 3:
				return "Float";
			case 4:
				return "Fixed";
			default:
				return "N/A";
		}
	}
}
