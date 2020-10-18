package com.dtucar.dynamo;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

public class MainActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        ActionResolver.getInstance().initialize(this);
		launchControl(null);
    }

    public void launchControl(View view) {
        Intent intent = new Intent(this, ControlActivity.class);
        startActivity(intent);
    }

    public void launchMaps(View view) {
        Intent intent = new Intent(this, MapsActivity.class);
        startActivity(intent);
    }
}
