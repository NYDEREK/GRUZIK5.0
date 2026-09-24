package com.snyde.robotapp;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.RectF;
import android.view.View;

import java.util.ArrayList;
import java.util.List;

class RouteMapView extends View {
    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final ArrayList<RoutePoint> recorded = new ArrayList<>();
    private final ArrayList<RoutePoint> optimized = new ArrayList<>();
    private final ArrayList<RoutePoint> robotOptimized = new ArrayList<>();

    RouteMapView(Context context) {
        super(context);
        setMinimumHeight(dp(280));
        textPaint.setTextSize(dp(12));
        textPaint.setColor(Color.rgb(202, 211, 223));
    }

    void setRoutes(List<RoutePoint> recordedRoute, List<RoutePoint> optimizedRoute, List<RoutePoint> robotOptimizedRoute) {
        recorded.clear();
        optimized.clear();
        robotOptimized.clear();
        if (recordedRoute != null) {
            recorded.addAll(recordedRoute);
        }
        if (optimizedRoute != null) {
            optimized.addAll(optimizedRoute);
        }
        if (robotOptimizedRoute != null) {
            robotOptimized.addAll(robotOptimizedRoute);
        }
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.rgb(18, 21, 26));
        canvas.drawRoundRect(new RectF(0, 0, getWidth(), getHeight()), dp(8), dp(8), paint);

        drawGrid(canvas);

        Bounds bounds = boundsForRoutes();
        if (bounds == null) {
            drawEmpty(canvas);
            return;
        }

        SpeedRange optimizedSpeed = speedRangeFor(optimized);
        drawRoute(canvas, recorded, bounds, Color.rgb(117, 129, 145), 3.0f, false, null);
        drawRoute(canvas, robotOptimized, bounds, Color.rgb(245, 158, 11), 3.0f, true, null);
        drawRoute(canvas, optimized, bounds, Color.rgb(20, 184, 166), 4.5f, false, optimizedSpeed);
        drawLegend(canvas, optimizedSpeed);
    }

    private void drawGrid(Canvas canvas) {
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(1.0f);
        paint.setColor(Color.rgb(39, 45, 54));
        paint.setPathEffect(null);

        int lines = 5;
        for (int i = 1; i < lines; i++) {
            float x = getWidth() * i / (float) lines;
            float y = getHeight() * i / (float) lines;
            canvas.drawLine(x, dp(12), x, getHeight() - dp(12), paint);
            canvas.drawLine(dp(12), y, getWidth() - dp(12), y, paint);
        }
    }

    private void drawEmpty(Canvas canvas) {
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.rgb(148, 163, 184));
        paint.setTextSize(dp(14));
        paint.setTextAlign(Paint.Align.CENTER);
        canvas.drawText("Brak mapy", getWidth() / 2.0f, getHeight() / 2.0f, paint);
    }

    private void drawLegend(Canvas canvas, SpeedRange optimizedSpeed) {
        float x = dp(14);
        float y = dp(22);
        legendItem(canvas, x, y, Color.rgb(117, 129, 145), "zapisana");
        legendItem(canvas, x + dp(92), y, speedColor(0.55f), "optymalna");
        legendItem(canvas, x + dp(190), y, Color.rgb(245, 158, 11), "map.txt");
        if (optimizedSpeed.valid) {
            drawSpeedLegend(canvas, optimizedSpeed);
        }
    }

    private void legendItem(Canvas canvas, float x, float y, int color, String label) {
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(3));
        paint.setColor(color);
        paint.setPathEffect(null);
        canvas.drawLine(x, y - dp(4), x + dp(20), y - dp(4), paint);
        canvas.drawText(label, x + dp(26), y, textPaint);
    }

    private void drawSpeedLegend(Canvas canvas, SpeedRange speedRange) {
        float left = dp(14);
        float top = getHeight() - dp(24);
        float width = Math.min(dp(190), getWidth() - dp(28));
        float step = width / 6.0f;

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(5));
        paint.setPathEffect(null);
        for (int i = 0; i < 6; i++) {
            float t = i / 5.0f;
            paint.setColor(speedColor(t));
            canvas.drawLine(left + (step * i), top, left + (step * (i + 1)), top, paint);
        }

        String label = String.format(java.util.Locale.US, "%.0f-%.0f speed", speedRange.min, speedRange.max);
        canvas.drawText(label, left + width + dp(8), top + dp(4), textPaint);
    }

    private void drawRoute(Canvas canvas, ArrayList<RoutePoint> route, Bounds bounds, int color, float widthDp, boolean dashed, SpeedRange speedRange) {
        if (route.size() < 2) {
            return;
        }

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeCap(Paint.Cap.ROUND);
        paint.setStrokeJoin(Paint.Join.ROUND);
        paint.setStrokeWidth(dpFloat(widthDp));
        paint.setPathEffect(dashed ? new DashPathEffect(new float[]{dp(10), dp(7)}, 0) : null);

        for (int i = 1; i < route.size(); i++) {
            RoutePoint a = route.get(i - 1);
            RoutePoint b = route.get(i);
            paint.setColor(speedRange == null ? color : speedColorForValue(b.speed, speedRange));
            canvas.drawLine(toScreenX(a.x, bounds), toScreenY(a.y, bounds),
                    toScreenX(b.x, bounds), toScreenY(b.y, bounds), paint);
        }
        paint.setPathEffect(null);
    }

    private int speedColorForValue(float speed, SpeedRange speedRange) {
        if (!speedRange.valid || speedRange.max <= speedRange.min) {
            return Color.rgb(20, 184, 166);
        }
        float t = Math.max(0.0f, Math.min(1.0f, (speed - speedRange.min) / (speedRange.max - speedRange.min)));
        return speedColor(t);
    }

    private int speedColor(float t) {
        t = Math.max(0.0f, Math.min(1.0f, t));
        int r;
        int g;
        int b;
        if (t < 0.5f) {
            float u = t * 2.0f;
            r = Math.round(14 + (132 * u));
            g = Math.round(165 + (232 - 165) * u);
            b = Math.round(233 + (112 - 233) * u);
        } else {
            float u = (t - 0.5f) * 2.0f;
            r = Math.round(146 + (239 - 146) * u);
            g = Math.round(232 + (68 - 232) * u);
            b = Math.round(112 + (68 - 112) * u);
        }
        return Color.rgb(r, g, b);
    }

    private SpeedRange speedRangeFor(ArrayList<RoutePoint> route) {
        SpeedRange range = new SpeedRange();
        for (RoutePoint point : route) {
            if (point.speed > 0.0f) {
                range.min = Math.min(range.min, point.speed);
                range.max = Math.max(range.max, point.speed);
                range.valid = true;
            }
        }
        return range;
    }

    private Bounds boundsForRoutes() {
        Bounds bounds = new Bounds();
        boolean hasPoint = false;
        hasPoint |= include(recorded, bounds);
        hasPoint |= include(optimized, bounds);
        hasPoint |= include(robotOptimized, bounds);
        if (!hasPoint) {
            return null;
        }

        float dx = bounds.maxX - bounds.minX;
        float dy = bounds.maxY - bounds.minY;
        if (dx < 0.01f) {
            bounds.minX -= 0.05f;
            bounds.maxX += 0.05f;
        }
        if (dy < 0.01f) {
            bounds.minY -= 0.05f;
            bounds.maxY += 0.05f;
        }
        return bounds;
    }

    private boolean include(ArrayList<RoutePoint> route, Bounds bounds) {
        boolean hasPoint = false;
        for (RoutePoint point : route) {
            bounds.minX = Math.min(bounds.minX, point.x);
            bounds.maxX = Math.max(bounds.maxX, point.x);
            bounds.minY = Math.min(bounds.minY, point.y);
            bounds.maxY = Math.max(bounds.maxY, point.y);
            hasPoint = true;
        }
        return hasPoint;
    }

    private float toScreenX(float x, Bounds bounds) {
        float padding = dp(24);
        return padding + ((x - bounds.minX) / (bounds.maxX - bounds.minX)) * (getWidth() - padding * 2.0f);
    }

    private float toScreenY(float y, Bounds bounds) {
        float padding = dp(30);
        return getHeight() - padding - ((y - bounds.minY) / (bounds.maxY - bounds.minY)) * (getHeight() - padding * 2.0f);
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    private float dpFloat(float value) {
        return value * getResources().getDisplayMetrics().density;
    }

    private static class Bounds {
        float minX = Float.MAX_VALUE;
        float maxX = -Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;
        float maxY = -Float.MAX_VALUE;
    }

    private static class SpeedRange {
        boolean valid;
        float min = Float.MAX_VALUE;
        float max = -Float.MAX_VALUE;
    }
}
