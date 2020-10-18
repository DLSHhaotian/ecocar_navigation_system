package com.dtucar.dynamo;

import java.util.ArrayList;

import android.app.Activity;
import android.os.Bundle;
import android.support.v4.app.FragmentActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.ViewConfiguration;
import android.view.ViewGroup;
import android.view.Window;
import android.view.WindowManager;
import android.widget.FrameLayout;

public class MultiTouchActivity extends FragmentActivity implements OnTouchListener {

	private View parent;

	private final ArrayList[] recentTouchedViewsIndex = new ArrayList[10];

	private final ArrayList[] downTouchedViewsIndex = new ArrayList[10];

	private final ArrayList<View> moveOutsideEnabledViews = new ArrayList<View>();

	private int mTouchSlop = 24;

	public void addMoveOutsideEnabledViews(final View view) {
		moveOutsideEnabledViews.add(view);
	}

	private void dealEvent(final int actionPointerIndex,
			final MotionEvent event, final View eventView,
			final int actionResolved) {
		int rawX, rawY;
		final int location[] = { 0, 0 };
		eventView.getLocationOnScreen(location);
		// Log.v("tag", location + "");
		rawX = (int) event.getX(actionPointerIndex) + location[0];
		rawY = (int) event.getY(actionPointerIndex) + location[1];

		final int actionPointerID = event.getPointerId(actionPointerIndex);
		ArrayList<View> hoverViews = getTouchedViews(rawX, rawY);

		if (actionResolved == MotionEvent.ACTION_DOWN) {
			downTouchedViewsIndex[actionPointerID] = (ArrayList<View>) hoverViews
					.clone();
		}
		// deletes all views which where not clicked on ActionDown
		if (downTouchedViewsIndex[actionPointerID] != null) {
			final ArrayList<View> tempViews = (ArrayList<View>) hoverViews
					.clone();
			tempViews.removeAll(downTouchedViewsIndex[actionPointerID]);
			hoverViews.removeAll(tempViews);
		}

		if (recentTouchedViewsIndex[actionPointerID] != null) {
			final ArrayList<View> recentTouchedViews = recentTouchedViewsIndex[actionPointerID];

			final ArrayList<View> shouldTouchViews = (ArrayList<View>) hoverViews
					.clone();
			if (!shouldTouchViews.containsAll(recentTouchedViews)) {
				shouldTouchViews.removeAll(recentTouchedViews);
				shouldTouchViews.addAll(recentTouchedViews);

				final ArrayList<View> outsideTouchedViews = (ArrayList<View>) shouldTouchViews
						.clone();
				outsideTouchedViews.removeAll(hoverViews);
			}

			recentTouchedViewsIndex[actionPointerID] = hoverViews;
			hoverViews = shouldTouchViews;
		} else {
			recentTouchedViewsIndex[actionPointerID] = hoverViews;
		}

		if (actionResolved == MotionEvent.ACTION_UP) {
			recentTouchedViewsIndex[actionPointerID] = null;
			downTouchedViewsIndex[actionPointerID] = null;
		}

		dumpEvent(event);
		for (final View view : hoverViews) {
			int x, y;
			view.getLocationOnScreen(location);
			x = rawX - location[0];
			y = rawY - location[1];

			// View does not recognize that the Pointer is
			// outside if the Pointer is not far away (>mTouchSlop)
			if (recentTouchedViewsIndex[actionPointerID] != null) {
				if (pointInView(x, y, mTouchSlop, view.getWidth(),
						view.getHeight())) {
					// Log.v("tag", "added because < mTouchSlop");

					if (!recentTouchedViewsIndex[actionPointerID]
							.contains(view)) {
						recentTouchedViewsIndex[actionPointerID].add(view);
					}
				} else if (moveOutsideEnabledViews.contains(view)) {
					Log.v("tag", "outside but gets event");
					recentTouchedViewsIndex[actionPointerID].add(view);
				}
			}
			final MotionEvent me = MotionEvent.obtain(event.getDownTime(),
					event.getEventTime(), actionResolved, x, y,
					event.getPressure(actionPointerIndex),
					event.getPressure(actionPointerIndex),
					event.getMetaState(), event.getXPrecision(),
					event.getYPrecision(), event.getDeviceId(),
					event.getEdgeFlags());
			me.setLocation(x, y);

			if (!me.equals(event)) {
				// deals the Event
				view.onTouchEvent(me);
			}

			// debug
			if (actionResolved == MotionEvent.ACTION_MOVE) {
				Log.v("tag",
						"#" + actionPointerIndex + " Rawx:" + rawX + " rawy:"
								+ rawY + " x:" + x + " y:" + y + " "
								+ view.toString());
			}
		}

	}

	private void dumpEvent(final MotionEvent event) {
		final String names[] = { "DOWN", "UP", "MOVE", "CANCEL", "OUTSIDE",
				"POINTER_DOWN", "POINTER_UP", "7?", "8?", "9?" };
		final StringBuilder sb = new StringBuilder();
		final int action = event.getAction();
		final int actionCode = action & MotionEvent.ACTION_MASK;
		sb.append("event ACTION_").append(names[actionCode]);
		if (actionCode == MotionEvent.ACTION_POINTER_DOWN
				|| actionCode == MotionEvent.ACTION_POINTER_UP) {
			sb.append("(pid ").append(
					action >> MotionEvent.ACTION_POINTER_ID_SHIFT);
			sb.append(")");
		}
		sb.append("[");
		for (int i = 0; i < event.getPointerCount(); i++) {
			sb.append("#").append(i);
			sb.append("(pid ").append(event.getPointerId(i));
			sb.append(")=").append((int) event.getX(i));
			sb.append(",").append((int) event.getY(i));
			if (i + 1 < event.getPointerCount()) {
				sb.append(";");
			}
		}
		sb.append("]");
		Log.d("tag", sb.toString());
	}

	private ArrayList<View> getChildViews(final View view) {
		final ArrayList<View> views = new ArrayList<View>();
		if (view instanceof ViewGroup) {
			final ViewGroup v = ((ViewGroup) view);
			if (v.getChildCount() > 0) {
				for (int i = 0; i < v.getChildCount(); i++) {
					views.add(v.getChildAt(i));
				}

			}
		}
		return views;
	}

	private ArrayList<View> getTouchedViews(final int x, final int y) {

		final ArrayList<View> touchedViews = new ArrayList<View>();
		final ArrayList<View> possibleViews = new ArrayList<View>();

		if (parent instanceof ViewGroup) {
			possibleViews.add(parent);
			for (int i = 0; i < possibleViews.size(); i++) {
				final View view = possibleViews.get(i);

				final int location[] = { 0, 0 };
				view.getLocationOnScreen(location);

				if (((view.getHeight() + location[1] >= y)
						& (view.getWidth() + location[0] >= x)
						& (view.getLeft() <= x) & (view.getTop() <= y))
						|| view instanceof FrameLayout) {
					touchedViews.add(view);
					possibleViews.addAll(getChildViews(view));
				}

			}
		}

		return touchedViews;

	}

	@Override
	public void onCreate(final Bundle instance) {
		super.onCreate(instance);
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
		getWindow().clearFlags(
				WindowManager.LayoutParams.FLAG_FORCE_NOT_FULLSCREEN);
		parent = findViewById(android.R.id.content).getRootView();
		parent.setOnTouchListener(this);
		mTouchSlop = ViewConfiguration.get(getApplicationContext())
				.getScaledTouchSlop();
	}

	@Override
	public boolean onTouch(final View v, final MotionEvent event) {

		// index of the pointer which starts this Event
		final int actionPointerIndex = event.getActionIndex();

		// resolve the action as a basic type (up, down or move)
		int actionResolved = event.getAction() & MotionEvent.ACTION_MASK;
		if (actionResolved < 7 && actionResolved > 4) {
			actionResolved = actionResolved - 5;
		}

		if (actionResolved == MotionEvent.ACTION_MOVE) {
			for (int ptrIndex = 0; ptrIndex < event.getPointerCount(); ptrIndex++) {
				// only one event for all move events.
				dealEvent(ptrIndex, event, v, actionResolved);
				Log.v("tag", "move" + ptrIndex);
			}

		} else {
			dealEvent(actionPointerIndex, event, v, actionResolved);
		}

		return true;
	}

	private boolean pointInView(final float localX, final float localY,
			final float slop, final float width, final float height) {
		return localX >= -slop && localY >= -slop && localX < ((width) + slop)
				&& localY < ((height) + slop);
	}
}
