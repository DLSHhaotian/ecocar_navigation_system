package com.dtucar.dynamo;

import android.os.Handler;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;
import android.view.MenuItem;
import android.view.WindowManager;

import com.dtucar.dynamo.communication.OnMessageReceivedListener;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.VisibleRegion;

public class MapsActivity extends FragmentActivity implements OnMapReadyCallback, OnMessageReceivedListener {

    private GoogleMap mMap;
    private Marker marker;
    private Handler handler;
    private LatLng position;
    private static final float ZOOM_LEVEL = 16f;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_maps);
        // Obtain the SupportMapFragment and get notified when the map is ready to be used.
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;

        // Add a marker in Sydney and move the camera
        position = new LatLng(55.7834892, 12.5226897);
        MarkerOptions markerOptions = new MarkerOptions().position(position).title("Dynamo");
        marker = mMap.addMarker(markerOptions);
        mMap.moveCamera(CameraUpdateFactory.newLatLng(position));
        mMap.moveCamera(CameraUpdateFactory.zoomTo(ZOOM_LEVEL));

        ActionResolver.getInstance().registerMessageReceivedListener(this);
    }

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

    @Override
    public void onStop() {
        ActionResolver.getInstance().unregisterMessageReceivedListener(this);
        super.onStop();
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch(item.getItemId()) {
            case android.R.id.home:
                onBackPressed();
                break;
        }
        return true;
    }

    float i = 12.5226897f;

    @Override
    public void onMessageReceived(String message) {
        position = new LatLng(55.7834892, i += 0.001); // TODO get position from message
        LatLngBounds latLngBounds = mMap.getProjection().getVisibleRegion().latLngBounds;
        if(!latLngBounds.contains(position)) {
            mMap.moveCamera(CameraUpdateFactory.newLatLng(position));
        }
        marker.setPosition(position);
    }
}
