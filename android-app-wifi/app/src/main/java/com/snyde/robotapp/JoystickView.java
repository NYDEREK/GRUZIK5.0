package com.snyde.robotapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.view.MotionEvent;
import android.view.View;

class JoystickView extends View {
    interface Listener {
        void onMove(float forward, float turn, boolean active);
    }

    private static final int SURFACE = Color.rgb(24, 24, 26);
    private static final int LINE = Color.rgb(52, 52, 57);
    private static final int KNOB_IDLE = Color.rgb(44, 44, 48);
    private static final int KNOB_ACTIVE = Color.rgb(236, 236, 238);

    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF bounds = new RectF();
    private Listener listener;
    private float knobX;
    private float knobY;
    private boolean active;

    JoystickView(Context context) {
        super(context);
        setMinimumHeight(dp(250));
    }

    void setListener(Listener listener) {
        this.listener = listener;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        float width = getWidth();
        float height = getHeight();
        float cx = width * 0.5f;
        float cy = height * 0.5f;
        float radius = Math.min(width, height) * 0.38f;
        float knobRadius = radius * 0.30f;

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(SURFACE);
        bounds.set(0, 0, width, height);
        canvas.drawRoundRect(bounds, dp(6), dp(6), paint);

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(1));
        paint.setColor(LINE);
        canvas.drawCircle(cx, cy, radius, paint);
        canvas.drawLine(cx - radius, cy, cx + radius, cy, paint);
        canvas.drawLine(cx, cy - radius, cx, cy + radius, paint);

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(active ? KNOB_ACTIVE : KNOB_IDLE);
        canvas.drawCircle(active ? knobX : cx, active ? knobY : cy, knobRadius, paint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        int action = event.getActionMasked();
        if (action == MotionEvent.ACTION_UP || action == MotionEvent.ACTION_CANCEL) {
            if (getParent() != null) {
                getParent().requestDisallowInterceptTouchEvent(false);
            }
            active = false;
            notifyMove(0.0f, 0.0f, false);
            invalidate();
            return true;
        }

        if (action == MotionEvent.ACTION_DOWN || action == MotionEvent.ACTION_MOVE) {
            if (getParent() != null) {
                getParent().requestDisallowInterceptTouchEvent(true);
            }
            active = true;
            float cx = getWidth() * 0.5f;
            float cy = getHeight() * 0.5f;
            float radius = Math.min(getWidth(), getHeight()) * 0.38f;
            float dx = event.getX() - cx;
            float dy = event.getY() - cy;
            float length = (float) Math.sqrt((dx * dx) + (dy * dy));
            if (length > radius && length > 0.0f) {
                dx = dx / length * radius;
                dy = dy / length * radius;
            }
            knobX = cx + dx;
            knobY = cy + dy;
            notifyMove(-dy / radius, dx / radius, true);
            invalidate();
            return true;
        }

        return true;
    }

    private void notifyMove(float forward, float turn, boolean isActive) {
        if (listener != null) {
            listener.onMove(forward, turn, isActive);
        }
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }
}
