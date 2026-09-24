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

    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private Listener listener;
    private float knobX;
    private float knobY;
    private boolean active;

    JoystickView(Context context) {
        super(context);
        setMinimumHeight(dp(250));
        setBackgroundColor(Color.TRANSPARENT);
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
        float knobRadius = radius * 0.34f;

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.rgb(18, 21, 26));
        canvas.drawRoundRect(new RectF(0, 0, width, height), dp(8), dp(8), paint);

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(2));
        paint.setColor(Color.rgb(58, 66, 78));
        canvas.drawCircle(cx, cy, radius, paint);
        canvas.drawLine(cx - radius, cy, cx + radius, cy, paint);
        canvas.drawLine(cx, cy - radius, cx, cy + radius, paint);

        float drawX = active ? knobX : cx;
        float drawY = active ? knobY : cy;
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(active ? Color.rgb(20, 184, 166) : Color.rgb(31, 36, 44));
        canvas.drawCircle(drawX, drawY, knobRadius, paint);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(2));
        paint.setColor(Color.rgb(148, 163, 184));
        canvas.drawCircle(drawX, drawY, knobRadius, paint);
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
            float turn = dx / radius;
            float forward = -dy / radius;
            notifyMove(forward, turn, true);
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
