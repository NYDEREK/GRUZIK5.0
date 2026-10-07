package com.snyde.robotapp;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.StateListDrawable;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.net.wifi.WifiNetworkSpecifier;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.Editable;
import android.text.InputType;
import android.text.TextWatcher;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowInsets;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.HorizontalScrollView;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Wi-Fi controller for GRUZIK5.0. The robot hosts the "GRUZIK5" access point
 * and a TCP text link on 192.168.4.1:3333; the phone keeps mobile data for
 * everything else.
 */
public class MainActivity extends Activity {
    private static final String ROBOT_SSID = "GRUZIK5";
    private static final String ROBOT_PASSWORD = "gruzik5.0";
    private static final String ROBOT_HOST = "192.168.4.1";
    private static final int ROBOT_PORT = 3333;
    private static final int REQUEST_WIFI = 41;

    private static final String PREFS = "gruzik5";
    private static final String KEY_PRESET_NAMES = "preset_names";
    private static final String KEY_CLEAN_SPEED = "clean.speed";
    private static final String KEY_JOYSTICK_SPEED = "joystick.speed";
    private static final String KEY_TAB = "tab";

    private static final int TAB_DRIVE = 0;
    private static final String[] TAB_NAMES = {"Drive", "Sensors", "Map", "Joystick", "Odometry", "Log"};

    private static final int STYLE_QUIET = 0;
    private static final int STYLE_PRIMARY = 1;
    private static final int STYLE_DANGER = 2;

    private static final int BG = Color.rgb(14, 14, 15);
    private static final int FIELD = Color.rgb(30, 30, 33);
    private static final int PRESSED = Color.rgb(44, 44, 48);
    private static final int LINE = Color.rgb(42, 42, 46);
    private static final int TEXT = Color.rgb(236, 236, 238);
    private static final int MUTED = Color.rgb(140, 140, 148);
    private static final int DANGER = Color.rgb(229, 72, 77);
    private static final int WARNING = Color.rgb(245, 165, 36);

    private static final int STATE_DISCONNECTED = 0;
    private static final int STATE_CONNECTING = 1;
    private static final int STATE_CONNECTED = 2;

    // Sensors sit SENSOR15 ... SENSOR0 from left to right on the robot.
    private static final String[] SENSOR_LABELS = {
            "15", "14", "13", "12", "11", "10", "9", "8", "7", "6", "5", "4", "3", "2", "1", "0"};

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final ExecutorService writeExecutor = Executors.newSingleThreadExecutor();
    private final LinkedHashMap<String, EditText> fields = new LinkedHashMap<>();
    private final LinkedHashMap<String, String> fieldDefaults = new LinkedHashMap<>();
    private final LinkedHashMap<String, EditText> mapFields = new LinkedHashMap<>();
    private final LinkedHashMap<String, String> mapFieldDefaults = new LinkedHashMap<>();
    private final LinkedHashMap<String, EditText> optimizerFields = new LinkedHashMap<>();
    private final LinkedHashMap<String, String> optimizerDefaults = new LinkedHashMap<>();
    private final ArrayList<String> presetNames = new ArrayList<>();
    private final ArrayList<TextView> tabLabels = new ArrayList<>();
    private final ArrayList<View> tabUnderlines = new ArrayList<>();
    private final ArrayList<View> pages = new ArrayList<>();
    private final ArrayList<RoutePoint> recordedRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> optimizedRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> robotRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> incomingRoute = new ArrayList<>();
    private final StringBuilder rxLineBuffer = new StringBuilder();

    // Wi-Fi link. connectionGeneration is bumped on every connect/disconnect so
    // late callbacks of an old attempt cannot touch the current connection.
    private ConnectivityManager connectivityManager;
    private volatile ConnectivityManager.NetworkCallback robotNetworkCallback;
    private volatile Socket socket;
    private volatile OutputStream outputStream;
    private volatile int connectionGeneration;
    private volatile long lastStartAckMs;
    private volatile boolean uploadRunning;
    private int connectionState = STATE_DISCONNECTED;
    private String streamMode = "off";
    private boolean presetTouched;
    private String incomingMapKind;

    private TextView connectionText;
    private Button connectButton;
    private TextView voltageText;
    private TextView warningText;
    private TextView logText;
    private Spinner presetSpinner;
    private EditText presetNameInput;
    private EditText cleanSpeedInput;
    private EditText joystickSpeedInput;
    private TextView joystickStatusText;
    private SensorBarsView sensorBars;
    private TextView sensorSummaryText;
    private Button sensorStreamButton;
    private Button odometryStreamButton;
    private RouteMapView routeMapView;
    private TextView routeInfoText;
    private TextView transferText;
    private TextView odomPoseText;
    private TextView odomSpeedText;
    private TextView odomYawText;

    private long lastManualSendMs;
    private int lastManualLeft;
    private int lastManualRight;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        connectivityManager = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
        buildUi();
        loadSettings();
        loadPresetNames();
        updateRouteUi();
        selectTab(prefs().getInt(KEY_TAB, TAB_DRIVE));
        connectRobot(false);
    }

    @Override
    protected void onPause() {
        saveSettings();
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        if (isConnected()) {
            sendLine("Manual=0,0\n", false);
            sendLine("Clean=0\n", false);
        }
        // Close only after the queued stop commands have been written.
        writeExecutor.execute(this::closeConnection);
        writeExecutor.shutdown();
        super.onDestroy();
    }

    // ---------------------------------------------------------------- UI

    private void buildUi() {
        ScrollView scrollView = new ScrollView(this);
        scrollView.setFillViewport(true);
        scrollView.setBackgroundColor(BG);
        scrollView.setOnApplyWindowInsetsListener(this::applySystemInsets);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(dp(16), dp(12), dp(16), dp(24));
        scrollView.addView(root, matchWrap());

        LinearLayout header = row();
        TextView title = label("GRUZIK5", 20, TEXT);
        title.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        voltageText = label("", 15, TEXT);
        voltageText.setGravity(Gravity.END);
        header.addView(title, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        header.addView(voltageText, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT));
        root.addView(header, matchWrap());

        LinearLayout connectionRow = row();
        connectionText = label("", 15, MUTED);
        connectionText.setGravity(Gravity.CENTER_VERTICAL);
        connectionText.setPadding(dp(12), 0, dp(12), 0);
        connectionText.setSingleLine(true);
        connectionText.setBackground(roundedDrawable(FIELD, FIELD));
        connectButton = button("Connect", STYLE_QUIET);
        connectionRow.addView(connectionText, new LinearLayout.LayoutParams(0, dp(48), 1f));
        LinearLayout.LayoutParams connectParams = new LinearLayout.LayoutParams(dp(116), dp(48));
        connectParams.setMargins(dp(8), 0, 0, 0);
        connectionRow.addView(connectButton, connectParams);
        root.addView(connectionRow, topMargin(8));
        connectButton.setOnClickListener(v -> onConnectClicked());

        LinearLayout driveRow = row();
        Button startButton = button("Start", STYLE_PRIMARY);
        Button stopButton = button("Stop", STYLE_DANGER);
        driveRow.addView(startButton, weighted(0, 4));
        driveRow.addView(stopButton, weighted(4, 0));
        root.addView(driveRow, topMargin(8));
        startButton.setOnClickListener(v -> startRobot());
        stopButton.setOnClickListener(v -> stopRobot());

        warningText = label("", 13, WARNING);
        warningText.setVisibility(View.GONE);
        root.addView(warningText, topMargin(6));

        root.addView(tabBar(), topMargin(10));
        root.addView(hairline());

        pages.add(buildDrivePage());
        pages.add(buildSensorsPage());
        pages.add(buildMapPage());
        pages.add(buildJoystickPage());
        pages.add(buildOdometryPage());
        pages.add(buildLogPage());
        for (View page : pages) {
            root.addView(page, topMargin(4));
        }

        setContentView(scrollView);
        setConnectionState(STATE_DISCONNECTED);
    }

    private WindowInsets applySystemInsets(View view, WindowInsets insets) {
        // Android 15+ always draws edge-to-edge, older versions lay out below the bars
        if (Build.VERSION.SDK_INT >= 35) {
            android.graphics.Insets bars = insets.getInsets(WindowInsets.Type.systemBars()
                    | WindowInsets.Type.displayCutout() | WindowInsets.Type.ime());
            view.setPadding(bars.left, bars.top, bars.right, bars.bottom);
        }
        return insets;
    }

    private View tabBar() {
        HorizontalScrollView scroller = new HorizontalScrollView(this);
        scroller.setHorizontalScrollBarEnabled(false);
        LinearLayout bar = row();
        for (int i = 0; i < TAB_NAMES.length; i++) {
            LinearLayout tab = new LinearLayout(this);
            tab.setOrientation(LinearLayout.VERTICAL);
            TextView name = label(TAB_NAMES[i], 15, MUTED);
            name.setPadding(0, dp(10), 0, dp(9));
            View underline = new View(this);
            underline.setBackgroundColor(TEXT);
            tab.addView(name, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT));
            tab.addView(underline, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(2)));
            final int index = i;
            tab.setOnClickListener(v -> selectTab(index));

            LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
            params.setMargins(0, 0, dp(22), 0);
            bar.addView(tab, params);
            tabLabels.add(name);
            tabUnderlines.add(underline);
        }
        scroller.addView(bar);
        return scroller;
    }

    private void selectTab(int tab) {
        if (tab < 0 || tab >= pages.size()) {
            tab = TAB_DRIVE;
        }
        for (int i = 0; i < pages.size(); i++) {
            boolean selected = i == tab;
            pages.get(i).setVisibility(selected ? View.VISIBLE : View.GONE);
            tabLabels.get(i).setTextColor(selected ? TEXT : MUTED);
            tabUnderlines.get(i).setVisibility(selected ? View.VISIBLE : View.INVISIBLE);
        }
        prefs().edit().putInt(KEY_TAB, tab).apply();
    }

    private View buildDrivePage() {
        LinearLayout page = page();

        page.addView(sectionLabel("PID"), matchWrap());
        page.addView(fieldRow(fields, fieldDefaults, "Kp", "Kp", "0.015", "Kd", "Kd", "0.55"), matchWrap());
        page.addView(fieldRow(fields, fieldDefaults, "Treshold", "Threshold", "3300", null, null, null), matchWrap());

        page.addView(sectionLabel("Speed"), matchWrap());
        page.addView(fieldRow(fields, fieldDefaults, "Base_speed", "Base", "125", "Max_speed", "Max", "200"), matchWrap());
        page.addView(fieldRow(fields, fieldDefaults, "Sharp_bend_speed_left", "Sharp bend left", "120",
                "Sharp_bend_speed_right", "Sharp bend right", "-75"), matchWrap());
        page.addView(fieldRow(fields, fieldDefaults, "Bend_speed_left", "Bend left", "120",
                "Bend_speed_right", "Bend right", "-75"), matchWrap());

        Button sendButton = button("Send values", STYLE_QUIET);
        page.addView(sendButton, topMargin(10));
        sendButton.setOnClickListener(v -> {
            if (!isConnected()) {
                showWarning("Not connected");
                return;
            }
            ArrayList<String> commands = fieldCommands(fields);
            new Thread(() -> sendLinesBlocking(commands)).start();
        });

        EditText thresholdInput = fields.get("Treshold");
        if (thresholdInput != null) {
            thresholdInput.addTextChangedListener(new SimpleWatcher(this::updateSensorThreshold));
        }

        page.addView(sectionLabel("Presets"), matchWrap());
        presetSpinner = new Spinner(this);
        page.addView(presetSpinner, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(48)));
        presetSpinner.setOnTouchListener((v, event) -> {
            if (event.getActionMasked() == MotionEvent.ACTION_UP) {
                presetTouched = true;
            }
            return false;
        });
        presetSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (presetTouched && position >= 0 && position < presetNames.size()) {
                    loadPreset(presetNames.get(position));
                }
                presetTouched = false;
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });

        LinearLayout presetRow = row();
        presetNameInput = input("Name");
        Button saveButton = button("Save", STYLE_QUIET);
        Button deleteButton = button("Delete", STYLE_QUIET);
        presetRow.addView(presetNameInput, new LinearLayout.LayoutParams(0, dp(48), 1f));
        presetRow.addView(saveButton, fixedWidth(88));
        presetRow.addView(deleteButton, fixedWidth(88));
        page.addView(presetRow, topMargin(8));
        saveButton.setOnClickListener(v -> saveCurrentPreset());
        deleteButton.setOnClickListener(v -> deleteSelectedPreset());

        page.addView(sectionLabel("Tire cleaning"), matchWrap());
        LinearLayout cleanRow = row();
        cleanRow.setGravity(Gravity.BOTTOM);
        cleanSpeedInput = numberInput("170");
        Button cleanButton = button("Hold to clean", STYLE_QUIET);
        cleanRow.addView(labeledInput("Speed", cleanSpeedInput), new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        LinearLayout.LayoutParams cleanParams = new LinearLayout.LayoutParams(0, dp(48), 1f);
        cleanParams.setMargins(dp(8), 0, 0, 0);
        cleanRow.addView(cleanButton, cleanParams);
        page.addView(cleanRow, matchWrap());
        cleanButton.setOnTouchListener((view, event) -> {
            int action = event.getActionMasked();
            if (action == MotionEvent.ACTION_DOWN) {
                view.setPressed(true);
                startTireCleaning();
            } else if (action == MotionEvent.ACTION_UP || action == MotionEvent.ACTION_CANCEL) {
                view.setPressed(false);
                sendInBackground("Clean=0\n");
            }
            return true;
        });

        return page;
    }

    private View buildSensorsPage() {
        LinearLayout page = page();

        sensorStreamButton = button("Start stream", STYLE_QUIET);
        page.addView(sensorStreamButton, topMargin(12));
        sensorStreamButton.setOnClickListener(v -> sendCommand("Telemetry", "debug".equals(streamMode) ? "off" : "debug"));

        sensorBars = new SensorBarsView(this);
        sensorBars.setLabels(SENSOR_LABELS);
        page.addView(sensorBars, sizedTopMargin(LinearLayout.LayoutParams.MATCH_PARENT, dp(280), 16));

        sensorSummaryText = label("No data", 14, MUTED);
        sensorSummaryText.setTypeface(Typeface.MONOSPACE);
        page.addView(sensorSummaryText, topMargin(10));

        TextView legend = label("Sensors as seen from above, left to right. Dashed line is the threshold.", 13, MUTED);
        page.addView(legend, topMargin(6));
        return page;
    }

    private View buildMapPage() {
        LinearLayout page = page();

        routeMapView = new RouteMapView(this);
        page.addView(routeMapView, sizedTopMargin(LinearLayout.LayoutParams.MATCH_PARENT, dp(300), 12));

        routeInfoText = label("", 13, MUTED);
        routeInfoText.setTypeface(Typeface.MONOSPACE);
        page.addView(routeInfoText, topMargin(8));
        transferText = label("", 13, TEXT);
        page.addView(transferText, topMargin(4));

        page.addView(sectionLabel("Run"), matchWrap());
        LinearLayout runRow = row();
        Button mappingButton = button("Start mapping", STYLE_QUIET);
        Button playbackButton = button("Run map", STYLE_QUIET);
        runRow.addView(mappingButton, weighted(0, 4));
        runRow.addView(playbackButton, weighted(4, 0));
        page.addView(runRow, topMargin(4));
        mappingButton.setOnClickListener(v -> startMappingRun());
        playbackButton.setOnClickListener(v -> startPlaybackRun());

        page.addView(sectionLabel("Route"), matchWrap());
        LinearLayout routeRow = row();
        Button recordedButton = button("Get recorded", STYLE_QUIET);
        Button robotMapButton = button("Get map.txt", STYLE_QUIET);
        routeRow.addView(recordedButton, weighted(0, 4));
        routeRow.addView(robotMapButton, weighted(4, 0));
        page.addView(routeRow, topMargin(4));
        Button clearButton = button("Clear view", STYLE_QUIET);
        page.addView(clearButton, topMargin(8));
        recordedButton.setOnClickListener(v -> sendCommand("MapDump", "recorded"));
        robotMapButton.setOnClickListener(v -> sendCommand("MapDump", "optimized"));
        clearButton.setOnClickListener(v -> {
            recordedRoute.clear();
            optimizedRoute.clear();
            robotRoute.clear();
            updateRouteUi();
        });

        page.addView(sectionLabel("Optimization"), matchWrap());
        page.addView(fieldRow(optimizerFields, optimizerDefaults, "opt.step", "Point step", "80",
                "opt.smooth", "Smoothing", "5"), matchWrap());
        page.addView(fieldRow(optimizerFields, optimizerDefaults, "opt.base", "Base speed", "40",
                "opt.gain", "Turn gain", "60"), matchWrap());
        page.addView(fieldRow(optimizerFields, optimizerDefaults, "opt.min", "Min speed", "35",
                "opt.max", "Max speed", "95"), matchWrap());
        LinearLayout optimizeRow = row();
        Button optimizeButton = button("Optimize", STYLE_QUIET);
        Button uploadButton = button("Upload map.txt", STYLE_QUIET);
        optimizeRow.addView(optimizeButton, weighted(0, 4));
        optimizeRow.addView(uploadButton, weighted(4, 0));
        page.addView(optimizeRow, topMargin(10));
        optimizeButton.setOnClickListener(v -> optimizeRecordedRoute(true));
        uploadButton.setOnClickListener(v -> uploadOptimizedRoute());

        page.addView(sectionLabel("Playback controller"), matchWrap());
        page.addView(fieldRow(mapFields, mapFieldDefaults, "MapP", "P", "90", "MapI", "I", "0"), matchWrap());
        page.addView(fieldRow(mapFields, mapFieldDefaults, "MapD", "D", "10", "MapSpeed", "Default speed", "80"), matchWrap());
        Button sendMapButton = button("Send values", STYLE_QUIET);
        page.addView(sendMapButton, topMargin(10));
        sendMapButton.setOnClickListener(v -> {
            if (!isConnected()) {
                showWarning("Not connected");
                return;
            }
            ArrayList<String> commands = fieldCommands(mapFields);
            new Thread(() -> sendLinesBlocking(commands)).start();
        });
        return page;
    }

    private View buildJoystickPage() {
        LinearLayout page = page();

        LinearLayout settingsRow = row();
        settingsRow.setGravity(Gravity.BOTTOM);
        joystickSpeedInput = numberInput("85");
        Button stopButton = button("Stop", STYLE_DANGER);
        settingsRow.addView(labeledInput("Max PWM", joystickSpeedInput), new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        LinearLayout.LayoutParams stopParams = new LinearLayout.LayoutParams(0, dp(48), 1f);
        stopParams.setMargins(dp(8), 0, 0, 0);
        settingsRow.addView(stopButton, stopParams);
        page.addView(settingsRow, topMargin(12));

        JoystickView joystick = new JoystickView(this);
        page.addView(joystick, sizedTopMargin(LinearLayout.LayoutParams.MATCH_PARENT, dp(300), 12));

        joystickStatusText = label("L 0   R 0", 14, MUTED);
        joystickStatusText.setTypeface(Typeface.MONOSPACE);
        joystickStatusText.setGravity(Gravity.CENTER);
        page.addView(joystickStatusText, topMargin(8));

        joystick.setListener((forward, turn, active) -> sendManualDrive(forward, turn, !active));
        stopButton.setOnClickListener(v -> sendManualStop());
        return page;
    }

    private View buildOdometryPage() {
        LinearLayout page = page();

        odometryStreamButton = button("Start stream", STYLE_QUIET);
        page.addView(odometryStreamButton, topMargin(12));
        odometryStreamButton.setOnClickListener(v -> sendCommand("Telemetry", "off".equals(streamMode) ? "odom" : "off"));

        page.addView(sectionLabel("Pose"), matchWrap());
        odomPoseText = readout();
        page.addView(odomPoseText, topMargin(4));

        page.addView(sectionLabel("Heading"), matchWrap());
        odomYawText = readout();
        page.addView(odomYawText, topMargin(4));

        page.addView(sectionLabel("Wheels"), matchWrap());
        odomSpeedText = readout();
        page.addView(odomSpeedText, topMargin(4));

        TextView legend = label("Pose is reset at every start.", 13, MUTED);
        page.addView(legend, topMargin(10));
        showOdometry(0, 0, 0, 0, 0, 0, 0, 0, 0);
        return page;
    }

    private View buildLogPage() {
        LinearLayout page = page();
        Button clearButton = button("Clear", STYLE_QUIET);
        page.addView(clearButton, topMargin(12));

        logText = label("", 12, TEXT);
        logText.setTypeface(Typeface.MONOSPACE);
        logText.setTextIsSelectable(true);
        page.addView(logText, topMargin(10));

        clearButton.setOnClickListener(v -> logText.setText(""));
        return page;
    }

    private LinearLayout fieldRow(Map<String, EditText> target, Map<String, String> defaults,
                                  String leftKey, String leftLabel, String leftDefault,
                                  String rightKey, String rightLabel, String rightDefault) {
        LinearLayout row = row();
        row.addView(field(target, defaults, leftKey, leftLabel, leftDefault), weighted(0, 4));
        if (rightKey != null) {
            row.addView(field(target, defaults, rightKey, rightLabel, rightDefault), weighted(4, 0));
        } else {
            row.addView(new View(this), weighted(4, 0));
        }
        return row;
    }

    private View field(Map<String, EditText> target, Map<String, String> defaults,
                       String key, String title, String defaultValue) {
        EditText input = numberInput(defaultValue);
        target.put(key, input);
        defaults.put(key, defaultValue);
        return labeledInput(title, input);
    }

    // ---------------------------------------------------------------- robot commands

    private void startRobot() {
        if (!isConnected()) {
            showWarning("Not connected");
            return;
        }
        ArrayList<String> commands = fieldCommands(fields);
        new Thread(() -> {
            long startRequestMs = System.currentTimeMillis();
            sendLine("Telemetry=off\n", true);
            sleepMs(40);
            sendLinesBlocking(commands);
            sleepMs(120);
            sendLine("StartNormal=1\n", true);
            fallbackStartIfNeeded(startRequestMs, "P");
        }).start();
    }

    private void startMappingRun() {
        if (!isConnected()) {
            showWarning("Not connected");
            return;
        }
        ArrayList<String> commands = fieldCommands(fields);
        transferText.setText("Starting mapping");
        new Thread(() -> {
            long startRequestMs = System.currentTimeMillis();
            sendLine("Telemetry=off\n", true);
            sleepMs(40);
            sendLinesBlocking(commands);
            sleepMs(120);
            sendLine("StartMapping=1\n", true);
            fallbackStartIfNeeded(startRequestMs, "M");
        }).start();
    }

    private void startPlaybackRun() {
        if (!isConnected()) {
            showWarning("Not connected");
            return;
        }
        if (optimizedRoute.isEmpty() && !optimizeRecordedRoute(true)) {
            return;
        }
        ArrayList<String> commands = fieldCommands(fields);
        commands.addAll(fieldCommands(mapFields));
        transferText.setText("Starting playback");
        new Thread(() -> {
            sendLine("Telemetry=off\n", true);
            sleepMs(40);
            sendLinesBlocking(commands);
            sleepMs(120);
            if (!uploadOptimizedRouteBlocking()) {
                return;
            }
            sleepMs(300);
            long startRequestMs = System.currentTimeMillis();
            sendLine("StartPlayback=1\n", true);
            fallbackStartIfNeeded(startRequestMs, "U");
        }).start();
    }

    // Firmware without the StartXxx commands: select the mode, then start it.
    private void fallbackStartIfNeeded(long startRequestMs, String mode) {
        sleepMs(650);
        if (lastStartAckMs >= startRequestMs) {
            return;
        }
        sendLine("Mode=" + mode + "\n", true);
        sleepMs(120);
        sendLine("Mode=Y\n", true);
    }

    private void stopRobot() {
        if (!isConnected()) {
            showWarning("Not connected");
            return;
        }
        new Thread(() -> {
            sendLine("Mode=N\n", true);
            sleepMs(25);
            sendLine("Manual=0,0\n", true);
            sleepMs(25);
            sendLine("Clean=0\n", true);
        }).start();
    }

    private ArrayList<String> fieldCommands(Map<String, EditText> source) {
        ArrayList<String> commands = new ArrayList<>();
        for (Map.Entry<String, EditText> entry : source.entrySet()) {
            String value = entry.getValue().getText().toString().trim().replace(',', '.');
            if (!value.isEmpty()) {
                commands.add(entry.getKey() + "=" + value + "\n");
            }
        }
        return commands;
    }

    private void sendLinesBlocking(List<String> commands) {
        for (String command : commands) {
            sendLine(command, true);
            sleepMs(30);
        }
    }

    private void startTireCleaning() {
        String speed = cleanSpeedInput.getText().toString().trim();
        String command = "CleanSpeed=" + (speed.isEmpty() ? "170" : speed) + "\n";
        new Thread(() -> {
            sendLine(command, true);
            sleepMs(25);
            sendLine("Clean=1\n", true);
        }).start();
    }

    private void sendManualDrive(float forward, float turn, boolean force) {
        int maxSpeed = clampInt(readInt(joystickSpeedInput, 85), 0, 250);
        int left = clampInt(Math.round((forward + turn) * maxSpeed), -250, 250);
        int right = clampInt(Math.round((forward - turn) * maxSpeed), -250, 250);

        if (!force) {
            long now = System.currentTimeMillis();
            if ((now - lastManualSendMs) < 70 &&
                    Math.abs(left - lastManualLeft) < 4 &&
                    Math.abs(right - lastManualRight) < 4) {
                return;
            }
            lastManualSendMs = now;
        }

        lastManualLeft = left;
        lastManualRight = right;
        joystickStatusText.setText(String.format(Locale.US, "L %d   R %d", left, right));
        sendLine(String.format(Locale.US, "Manual=%d,%d\n", left, right), false);
    }

    private void sendManualStop() {
        lastManualLeft = 0;
        lastManualRight = 0;
        lastManualSendMs = 0L;
        joystickStatusText.setText("L 0   R 0");
        sendInBackground("Manual=0,0\n");
    }

    private void sendCommand(String key, String value) {
        if (!isConnected()) {
            showWarning("Not connected");
            return;
        }
        sendInBackground(key + "=" + value + "\n");
    }

    private void sendInBackground(String command) {
        sendLine(command, true);
    }

    // Writes go through one background thread, so they keep their order and
    // never block the UI.
    private void sendLine(String command, boolean log) {
        OutputStream stream = outputStream;
        if (stream == null) {
            return;
        }
        int generation = connectionGeneration;
        if (log) {
            appendLog("> " + command);
        }
        byte[] bytes = command.getBytes(StandardCharsets.US_ASCII);
        try {
            writeExecutor.execute(() -> {
                try {
                    stream.write(bytes);
                    stream.flush();
                } catch (IOException ex) {
                    if (generation == connectionGeneration) {
                        mainHandler.post(() -> showWarning("Send failed: " + ex.getMessage()));
                        closeConnection();
                    }
                }
            });
        } catch (RuntimeException ignored) {
            // Executor already shut down while the activity is closing.
        }
    }

    // ---------------------------------------------------------------- robot replies

    private void handleIncomingText(String text) {
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            if (c == '\n') {
                String line = rxLineBuffer.toString().trim();
                rxLineBuffer.setLength(0);
                if (!line.isEmpty()) {
                    handleRobotLine(line);
                }
            } else if (c != '\r') {
                rxLineBuffer.append(c);
            }
        }
    }

    private void handleRobotLine(String line) {
        if (line.startsWith("DBG,")) {
            parseDebugTelemetry(line);
            return;
        }
        if (line.startsWith("ODOM,")) {
            parseOdomTelemetry(line);
            return;
        }
        if (line.startsWith("MAP,") && incomingMapKind != null) {
            RoutePoint point = parseRoutePoint(line.substring(4), incomingMapKind);
            if (point != null) {
                incomingRoute.add(point);
                if (incomingRoute.size() % 50 == 0) {
                    transferText.setText("Receiving " + incomingMapKind + ": " + incomingRoute.size() + " points");
                }
            }
            return;
        }

        appendLog("< " + line + "\n");

        if (line.startsWith("STARTING,") || "Start".equals(line)) {
            lastStartAckMs = System.currentTimeMillis();
            setStreamMode("off");
            hideWarning();
        } else if ("Stop".equals(line)) {
            hideWarning();
        }

        if (line.startsWith("TELEMETRY,")) {
            setStreamMode(line.substring("TELEMETRY,".length()).trim());
        }

        if (line.startsWith("MAP_BEGIN,")) {
            incomingMapKind = line.substring("MAP_BEGIN,".length()).trim();
            incomingRoute.clear();
            transferText.setText("Receiving " + incomingMapKind);
        } else if (line.startsWith("MAP_END") && incomingMapKind != null) {
            finishMapDownload();
        } else if (line.startsWith("MAP_AUTO_CLOSED")) {
            transferText.setText("Mapping closed the loop. Get recorded to see it.");
        } else if (line.startsWith("MAP_PLAYBACK_DONE")) {
            transferText.setText("Playback finished");
        } else if (line.startsWith("UPLOAD_")) {
            transferText.setText(line);
        } else if (line.startsWith("MAP_ERROR")) {
            transferText.setText(line);
        }

        if (line.startsWith("!") || line.contains("ERROR") || line.contains("stop_robot_first") ||
                line.startsWith("Stop robot")) {
            showWarning(line.replace("!", "").trim());
        }

        int batteryIndex = line.indexOf("Battery");
        if (batteryIndex >= 0) {
            String voltage = firstNumber(line.substring(batteryIndex));
            if (voltage != null) {
                voltageText.setText(voltage + " V");
            }
        }
    }

    // DBG,pos,active,last_end,<9 encoder/IMU fields>,S00..S15
    private void parseDebugTelemetry(String line) {
        String[] tokens = line.split(",");
        if (tokens.length < 13 + SensorBarsView.COUNT) {
            return;
        }
        float position = parseFloatToken(tokens, 1, 8500.0f);
        int active = Math.round(parseFloatToken(tokens, 2, 0.0f));
        int lastEnd = Math.round(parseFloatToken(tokens, 3, 0.0f));

        // Bars left to right are SENSOR15 ... SENSOR0. Position 16000 is the
        // left edge, 1000 the right edge.
        int[] values = new int[SensorBarsView.COUNT];
        for (int i = 0; i < SensorBarsView.COUNT; i++) {
            values[i] = Math.round(parseFloatToken(tokens, 13 + (SensorBarsView.COUNT - 1 - i), 0.0f));
        }
        float column = (16000.0f - position) / 1000.0f + 0.5f;

        if (!"debug".equals(streamMode)) {
            setStreamMode("debug");
        }
        sensorBars.setData(values, column, active > 0);
        if (active > 0) {
            sensorSummaryText.setText(String.format(Locale.US, "Position %-6.0f Active %-3d Last end %s",
                    position, active, lastEnd == 1 ? "right" : "left"));
        } else {
            sensorSummaryText.setText(String.format(Locale.US, "Line lost      Last end %s",
                    lastEnd == 1 ? "right" : "left"));
        }
    }

    // ODOM,x,y,yaw,distance,v_left,v_right,gyro,enc_yaw,gyro_yaw
    private void parseOdomTelemetry(String line) {
        String[] tokens = line.split(",");
        if (tokens.length < 10) {
            return;
        }
        showOdometry(parseFloatToken(tokens, 1, 0), parseFloatToken(tokens, 2, 0),
                parseFloatToken(tokens, 3, 0), parseFloatToken(tokens, 4, 0),
                parseFloatToken(tokens, 5, 0), parseFloatToken(tokens, 6, 0),
                parseFloatToken(tokens, 7, 0), parseFloatToken(tokens, 8, 0),
                parseFloatToken(tokens, 9, 0));
    }

    private void showOdometry(float x, float y, float yaw, float distance, float leftSpeed,
                              float rightSpeed, float gyro, float encoderYaw, float gyroYaw) {
        odomPoseText.setText(String.format(Locale.US, "X %8.3f m\nY %8.3f m\nDistance %8.3f m",
                x, y, distance));
        odomYawText.setText(String.format(Locale.US, "Yaw %7.1f deg\nEncoders %7.1f deg\nGyro %7.1f deg\nGyro rate %7.2f deg/s",
                yaw, encoderYaw, gyroYaw, gyro));
        odomSpeedText.setText(String.format(Locale.US, "Left %7.3f m/s\nRight %7.3f m/s",
                leftSpeed, rightSpeed));
    }

    private void setStreamMode(String mode) {
        streamMode = mode == null || mode.isEmpty() ? "off" : mode;
        if (sensorStreamButton != null) {
            sensorStreamButton.setText("debug".equals(streamMode) ? "Stop stream" : "Start stream");
        }
        if (odometryStreamButton != null) {
            odometryStreamButton.setText("off".equals(streamMode) ? "Start stream" : "Stop stream");
        }
    }

    private void updateSensorThreshold() {
        EditText input = fields.get("Treshold");
        if (sensorBars != null && input != null) {
            sensorBars.setThreshold(readFloat(input, 3300.0f));
        }
    }

    // ---------------------------------------------------------------- maps

    private void finishMapDownload() {
        boolean recorded = "recorded".equals(incomingMapKind);
        String message;
        if (recorded) {
            recordedRoute.clear();
            recordedRoute.addAll(incomingRoute);
            optimizedRoute.clear();
            boolean optimized = optimizeRecordedRoute(false);
            message = "Recorded route: " + incomingRoute.size() + " points" +
                    (optimized ? ", optimized: " + optimizedRoute.size() : "");
        } else {
            robotRoute.clear();
            robotRoute.addAll(incomingRoute);
            message = "map.txt: " + incomingRoute.size() + " points";
        }
        transferText.setText(message);
        incomingRoute.clear();
        incomingMapKind = null;
        updateRouteUi();
    }

    // GRUZIK.txt: x y sensor_x sensor_y speed error d_error; map.txt: x y speed.
    private RoutePoint parseRoutePoint(String value, String kind) {
        String[] tokens = value.trim().split("[,\\s]+");
        ArrayList<Float> numbers = new ArrayList<>();
        for (String token : tokens) {
            if (token.isEmpty()) {
                continue;
            }
            try {
                numbers.add(Float.parseFloat(token));
            } catch (NumberFormatException ignored) {
            }
        }
        if (numbers.size() < 2) {
            return null;
        }
        float speed = 0.0f;
        if ("recorded".equals(kind) && numbers.size() >= 5) {
            speed = numbers.get(4);
        } else if (numbers.size() >= 3) {
            speed = numbers.get(2);
        }
        return new RoutePoint(numbers.get(0), numbers.get(1), speed);
    }

    /**
     * Turns the recorded route into map.txt: resample every "point step"
     * points, smooth with a moving average, then give each point a speed that
     * drops with the local turn sharpness.
     */
    private boolean optimizeRecordedRoute(boolean showStatus) {
        if (recordedRoute.size() < 2) {
            if (showStatus) {
                showWarning("Get the recorded route first");
            }
            return false;
        }
        int step = clampInt(Math.round(readOptimizer("opt.step", 80)), 1, 500);
        int smoothing = clampInt(Math.round(readOptimizer("opt.smooth", 5)), 1, 101);
        float baseSpeed = readOptimizer("opt.base", 40);
        float gain = readOptimizer("opt.gain", 60);
        float minSpeed = readOptimizer("opt.min", 35);
        float maxSpeed = readOptimizer("opt.max", 95);
        if (maxSpeed < minSpeed) {
            float swap = maxSpeed;
            maxSpeed = minSpeed;
            minSpeed = swap;
        }

        boolean closed = isLoopClosed(recordedRoute);
        ArrayList<RoutePoint> source = new ArrayList<>(recordedRoute);
        if (closed) {
            source.remove(source.size() - 1);  // the closing point duplicates the start
        }
        if (source.size() < 2) {
            return false;
        }

        ArrayList<RoutePoint> sampled = new ArrayList<>();
        for (int i = 0; i < source.size(); i += step) {
            sampled.add(source.get(i));
        }
        RoutePoint last = source.get(source.size() - 1);
        if (distance(sampled.get(sampled.size() - 1), last) > 0.001f) {
            sampled.add(last);
        }

        ArrayList<RoutePoint> smoothed = smooth(sampled, smoothing, closed);
        optimizedRoute.clear();
        int n = smoothed.size();
        for (int i = 0; i < n; i++) {
            RoutePoint point = smoothed.get(i);
            RoutePoint previous = closed ? smoothed.get((i + n - 1) % n) : smoothed.get(Math.max(0, i - 1));
            RoutePoint next = closed ? smoothed.get((i + 1) % n) : smoothed.get(Math.min(n - 1, i + 1));
            float turn = turnAmount(previous, point, next);
            float speed = clampFloat(maxSpeed - gain * turn, minSpeed, maxSpeed);
            if (turn < 0.20f) {
                speed = Math.max(speed, Math.min(baseSpeed, maxSpeed));
            }
            optimizedRoute.add(new RoutePoint(point.x, point.y, speed));
        }
        updateRouteUi();
        if (showStatus) {
            transferText.setText("Optimized: " + optimizedRoute.size() + " points");
        }
        return true;
    }

    private ArrayList<RoutePoint> smooth(List<RoutePoint> input, int window, boolean closed) {
        ArrayList<RoutePoint> output = new ArrayList<>();
        int n = input.size();
        int radius = Math.max(0, Math.min(window, n) / 2);
        for (int i = 0; i < n; i++) {
            float x = 0.0f;
            float y = 0.0f;
            int count = 0;
            for (int j = i - radius; j <= i + radius; j++) {
                int index;
                if (closed) {
                    index = ((j % n) + n) % n;
                } else if (j < 0 || j >= n) {
                    continue;
                } else {
                    index = j;
                }
                x += input.get(index).x;
                y += input.get(index).y;
                count++;
            }
            output.add(new RoutePoint(x / count, y / count, 0.0f));
        }
        return output;
    }

    private boolean isLoopClosed(List<RoutePoint> route) {
        return route.size() > 2 && distance(route.get(0), route.get(route.size() - 1)) < 0.08f;
    }

    // 0 = straight, 1 = full reversal.
    private float turnAmount(RoutePoint previous, RoutePoint point, RoutePoint next) {
        double a = Math.atan2(point.y - previous.y, point.x - previous.x);
        double b = Math.atan2(next.y - point.y, next.x - point.x);
        double delta = b - a;
        while (delta > Math.PI) delta -= 2.0 * Math.PI;
        while (delta < -Math.PI) delta += 2.0 * Math.PI;
        return (float) (Math.abs(delta) / Math.PI);
    }

    private void uploadOptimizedRoute() {
        if (!isConnected()) {
            showWarning("Not connected");
            return;
        }
        if (optimizedRoute.isEmpty() && !optimizeRecordedRoute(true)) {
            return;
        }
        new Thread(this::uploadOptimizedRouteBlocking).start();
    }

    // MapUploadBegin=N, N x MapPoint=x,y,speed, MapUploadEnd=1.
    private boolean uploadOptimizedRouteBlocking() {
        if (uploadRunning) {
            mainHandler.post(() -> showWarning("Upload already running"));
            return false;
        }
        ArrayList<RoutePoint> route = new ArrayList<>(optimizedRoute);
        if (route.isEmpty() || !isConnected()) {
            return false;
        }
        uploadRunning = true;
        try {
            int count = route.size();
            mainHandler.post(() -> transferText.setText("Uploading map.txt: 0/" + count));
            sendLine("MapUploadBegin=" + count + "\n", true);
            sleepMs(160);
            for (int i = 0; i < count; i++) {
                RoutePoint p = route.get(i);
                sendLine(String.format(Locale.US, "MapPoint=%.3f,%.3f,%.3f\n", p.x, p.y, p.speed), false);
                int progress = i + 1;
                if (progress % 25 == 0 || progress == count) {
                    mainHandler.post(() -> transferText.setText("Uploading map.txt: " + progress + "/" + count));
                }
                sleepMs(35);
            }
            sendLine("MapUploadEnd=1\n", true);
            return true;
        } finally {
            uploadRunning = false;
        }
    }

    private void updateRouteUi() {
        if (routeMapView != null) {
            routeMapView.setRoutes(recordedRoute, optimizedRoute, robotRoute);
        }
        if (routeInfoText != null) {
            routeInfoText.setText(String.format(Locale.US,
                    "Recorded   %4d pts  %6.2f m\nOptimized  %4d pts  %6.2f m\nmap.txt    %4d pts",
                    recordedRoute.size(), routeLength(recordedRoute),
                    optimizedRoute.size(), routeLength(optimizedRoute), robotRoute.size()));
        }
    }

    private float routeLength(List<RoutePoint> route) {
        float length = 0.0f;
        for (int i = 1; i < route.size(); i++) {
            length += distance(route.get(i - 1), route.get(i));
        }
        return length;
    }

    private float distance(RoutePoint a, RoutePoint b) {
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        return (float) Math.sqrt(dx * dx + dy * dy);
    }

    private float readOptimizer(String key, float fallback) {
        EditText input = optimizerFields.get(key);
        return input == null ? fallback : readFloat(input, fallback);
    }

    // ---------------------------------------------------------------- Wi-Fi link

    private void onConnectClicked() {
        if (connectionState == STATE_DISCONNECTED) {
            connectRobot(false);
        } else {
            closeConnection();
        }
    }

    private void setConnectionState(int state) {
        connectionState = state;
        if (state == STATE_CONNECTED) {
            connectButton.setText("Disconnect");
            connectionText.setText(ROBOT_SSID + "  connected");
            connectionText.setTextColor(TEXT);
        } else if (state == STATE_CONNECTING) {
            connectButton.setText("Cancel");
            connectionText.setText(ROBOT_SSID + "  connecting");
            connectionText.setTextColor(MUTED);
        } else {
            connectButton.setText("Connect");
            connectionText.setText(ROBOT_SSID + "  offline");
            connectionText.setTextColor(MUTED);
            setStreamMode("off");
        }
    }

    private boolean isConnected() {
        return outputStream != null;
    }

    private ArrayList<String> missingWifiPermissions() {
        ArrayList<String> permissions = new ArrayList<>();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            if (checkSelfPermission(Manifest.permission.NEARBY_WIFI_DEVICES) != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.NEARBY_WIFI_DEVICES);
            }
        } else if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            permissions.add(Manifest.permission.ACCESS_FINE_LOCATION);
        }
        return permissions;
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_WIFI) {
            connectRobot(true);
        }
    }

    private void connectRobot(boolean permissionsAsked) {
        if (connectionState != STATE_DISCONNECTED) {
            return;
        }
        ArrayList<String> missing = missingWifiPermissions();
        if (!missing.isEmpty() && !permissionsAsked) {
            requestPermissions(missing.toArray(new String[0]), REQUEST_WIFI);
            return;
        }
        int generation = ++connectionGeneration;
        setConnectionState(STATE_CONNECTING);
        hideWarning();
        new Thread(() -> {
            // Phone already joined the robot network manually: use that network.
            for (Network network : connectivityManager.getAllNetworks()) {
                NetworkCapabilities caps = connectivityManager.getNetworkCapabilities(network);
                if (caps != null && caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)
                        && openSocket(network, generation, 900)) {
                    return;
                }
            }
            mainHandler.post(() -> requestRobotNetwork(generation));
        }).start();
    }

    // The robot network has no internet, so Android keeps mobile data as the
    // default route while only this app talks to the robot.
    private void requestRobotNetwork(int generation) {
        if (generation != connectionGeneration) {
            return;
        }
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.Q) {
            connectionFailed(generation, "Join " + ROBOT_SSID + " in the Wi-Fi settings first");
            return;
        }
        releaseRobotNetwork();
        WifiNetworkSpecifier specifier = new WifiNetworkSpecifier.Builder()
                .setSsid(ROBOT_SSID)
                .setWpa2Passphrase(ROBOT_PASSWORD)
                .build();
        NetworkRequest request = new NetworkRequest.Builder()
                .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
                .setNetworkSpecifier(specifier)
                .build();
        ConnectivityManager.NetworkCallback callback = new ConnectivityManager.NetworkCallback() {
            @Override
            public void onAvailable(Network network) {
                new Thread(() -> {
                    for (int attempt = 0; attempt < 6 && generation == connectionGeneration; attempt++) {
                        if (openSocket(network, generation, 2000)) {
                            return;
                        }
                        sleepMs(400);
                    }
                    connectionFailed(generation, "Robot does not answer");
                }).start();
            }

            @Override
            public void onUnavailable() {
                connectionFailed(generation, ROBOT_SSID + " not found");
            }

            @Override
            public void onLost(Network network) {
                if (generation == connectionGeneration) {
                    mainHandler.post(() -> {
                        closeConnection();
                        showWarning("Connection lost");
                    });
                }
            }
        };
        robotNetworkCallback = callback;
        try {
            connectivityManager.requestNetwork(request, callback);
        } catch (RuntimeException ex) {
            robotNetworkCallback = null;
            connectionFailed(generation, ex.getMessage());
        }
    }

    private boolean openSocket(Network network, int generation, int timeoutMs) {
        Socket newSocket = null;
        try {
            newSocket = network.getSocketFactory().createSocket();
            newSocket.connect(new InetSocketAddress(ROBOT_HOST, ROBOT_PORT), timeoutMs);
            newSocket.setTcpNoDelay(true);
            newSocket.setKeepAlive(true);
            if (generation != connectionGeneration) {
                newSocket.close();
                return true;
            }
            socket = newSocket;
            outputStream = newSocket.getOutputStream();
            startReadThread(newSocket.getInputStream(), generation);
            mainHandler.post(() -> setConnectionState(STATE_CONNECTED));
            return true;
        } catch (IOException ex) {
            if (newSocket != null) {
                try {
                    newSocket.close();
                } catch (IOException ignored) {
                }
            }
            return false;
        }
    }

    private void startReadThread(InputStream inputStream, int generation) {
        new Thread(() -> {
            byte[] buffer = new byte[1024];
            try {
                int len;
                while ((len = inputStream.read(buffer)) > 0) {
                    String text = new String(buffer, 0, len, StandardCharsets.US_ASCII);
                    mainHandler.post(() -> handleIncomingText(text));
                }
            } catch (IOException ignored) {
            }
            if (generation == connectionGeneration) {
                mainHandler.post(() -> {
                    closeConnection();
                    showWarning("Connection lost");
                });
            }
        }).start();
    }

    private void connectionFailed(int generation, String message) {
        if (generation != connectionGeneration) {
            return;
        }
        releaseRobotNetwork();
        mainHandler.post(() -> {
            setConnectionState(STATE_DISCONNECTED);
            if (message != null) {
                showWarning(message);
            }
        });
    }

    private void closeConnection() {
        connectionGeneration++;
        Socket current = socket;
        socket = null;
        outputStream = null;
        if (current != null) {
            try {
                current.close();
            } catch (IOException ignored) {
            }
        }
        releaseRobotNetwork();
        mainHandler.post(() -> setConnectionState(STATE_DISCONNECTED));
    }

    private void releaseRobotNetwork() {
        ConnectivityManager.NetworkCallback callback = robotNetworkCallback;
        robotNetworkCallback = null;
        if (callback != null) {
            try {
                connectivityManager.unregisterNetworkCallback(callback);
            } catch (RuntimeException ignored) {
            }
        }
    }

    // ---------------------------------------------------------------- presets and settings

    private void loadSettings() {
        SharedPreferences store = prefs();
        loadFields(store, fields, fieldDefaults, "current.");
        loadFields(store, mapFields, mapFieldDefaults, "map.");
        loadFields(store, optimizerFields, optimizerDefaults, "");
        cleanSpeedInput.setText(store.getString(KEY_CLEAN_SPEED, "170"));
        joystickSpeedInput.setText(store.getString(KEY_JOYSTICK_SPEED, "85"));
        updateSensorThreshold();
    }

    private void loadFields(SharedPreferences store, Map<String, EditText> source,
                            Map<String, String> defaults, String prefix) {
        for (Map.Entry<String, EditText> entry : source.entrySet()) {
            entry.getValue().setText(store.getString(prefix + entry.getKey(), defaults.get(entry.getKey())));
        }
    }

    private void saveSettings() {
        SharedPreferences.Editor editor = prefs().edit();
        saveFields(editor, fields, "current.");
        saveFields(editor, mapFields, "map.");
        saveFields(editor, optimizerFields, "");
        editor.putString(KEY_CLEAN_SPEED, cleanSpeedInput.getText().toString().trim());
        editor.putString(KEY_JOYSTICK_SPEED, joystickSpeedInput.getText().toString().trim());
        editor.apply();
    }

    private void saveFields(SharedPreferences.Editor editor, Map<String, EditText> source, String prefix) {
        for (Map.Entry<String, EditText> entry : source.entrySet()) {
            editor.putString(prefix + entry.getKey(), entry.getValue().getText().toString().trim());
        }
    }

    private void loadPresetNames() {
        presetNames.clear();
        String joined = prefs().getString(KEY_PRESET_NAMES, "");
        if (joined != null) {
            for (String part : joined.split("\\|", -1)) {
                String name = part.trim();
                if (!name.isEmpty()) {
                    presetNames.add(name);
                }
            }
        }
        updatePresetSpinner();
    }

    private void saveCurrentPreset() {
        String name = presetNameInput.getText().toString().trim().replace("|", "");
        if (name.isEmpty()) {
            showWarning("Preset name is empty");
            return;
        }

        SharedPreferences.Editor editor = prefs().edit();
        for (Map.Entry<String, EditText> entry : fields.entrySet()) {
            editor.putString(presetKey(name, entry.getKey()), entry.getValue().getText().toString().trim());
        }
        if (!presetNames.contains(name)) {
            presetNames.add(name);
        }
        editor.putString(KEY_PRESET_NAMES, joinPresetNames());
        editor.apply();
        updatePresetSpinner();
        presetSpinner.setSelection(presetNames.indexOf(name));
    }

    private void loadPreset(String name) {
        presetNameInput.setText(name);
        for (Map.Entry<String, EditText> entry : fields.entrySet()) {
            String current = entry.getValue().getText().toString();
            entry.getValue().setText(prefs().getString(presetKey(name, entry.getKey()), current));
        }
    }

    private void deleteSelectedPreset() {
        int index = presetSpinner.getSelectedItemPosition();
        if (index < 0 || index >= presetNames.size()) {
            return;
        }
        String name = presetNames.get(index);
        new AlertDialog.Builder(this)
                .setTitle("Delete preset")
                .setMessage(name)
                .setPositiveButton("Delete", (dialog, which) -> {
                    SharedPreferences.Editor editor = prefs().edit();
                    for (String key : fields.keySet()) {
                        editor.remove(presetKey(name, key));
                    }
                    presetNames.remove(name);
                    editor.putString(KEY_PRESET_NAMES, joinPresetNames());
                    editor.apply();
                    updatePresetSpinner();
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void updatePresetSpinner() {
        ArrayList<String> names = new ArrayList<>(presetNames);
        if (names.isEmpty()) {
            names.add("No presets");
        }
        presetSpinner.setAdapter(new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, names));
    }

    private String joinPresetNames() {
        StringBuilder builder = new StringBuilder();
        for (String name : presetNames) {
            if (builder.length() > 0) {
                builder.append('|');
            }
            builder.append(name);
        }
        return builder.toString();
    }

    private String presetKey(String presetName, String fieldName) {
        return "preset." + presetName + "." + fieldName;
    }

    private SharedPreferences prefs() {
        return getSharedPreferences(PREFS, Context.MODE_PRIVATE);
    }

    // ---------------------------------------------------------------- messages

    private void showWarning(String message) {
        warningText.setText(message);
        warningText.setVisibility(View.VISIBLE);
    }

    private void hideWarning() {
        warningText.setVisibility(View.GONE);
    }

    private void appendLog(String value) {
        if (Looper.myLooper() != Looper.getMainLooper()) {
            mainHandler.post(() -> appendLog(value));
            return;
        }
        String next = logText.getText().toString() + value;
        if (next.length() > 8000) {
            next = next.substring(next.length() - 8000);
        }
        logText.setText(next);
    }

    // ---------------------------------------------------------------- view helpers

    private LinearLayout page() {
        LinearLayout page = new LinearLayout(this);
        page.setOrientation(LinearLayout.VERTICAL);
        return page;
    }

    private TextView label(String text, int sp, int color) {
        TextView label = new TextView(this);
        label.setText(text);
        label.setTextSize(sp);
        label.setTextColor(color);
        return label;
    }

    private TextView sectionLabel(String text) {
        TextView label = label(text, 14, TEXT);
        label.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        label.setPadding(0, dp(20), 0, dp(2));
        return label;
    }

    private TextView readout() {
        TextView value = label("", 15, TEXT);
        value.setTypeface(Typeface.MONOSPACE);
        value.setLineSpacing(dp(2), 1.0f);
        return value;
    }

    private View hairline() {
        View line = new View(this);
        line.setBackgroundColor(LINE);
        line.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(1)));
        return line;
    }

    private EditText input(String hint) {
        EditText input = new EditText(this);
        input.setHint(hint);
        input.setTextSize(15);
        input.setTextColor(TEXT);
        input.setHintTextColor(MUTED);
        input.setSingleLine(true);
        input.setPadding(dp(12), 0, dp(12), 0);
        input.setBackground(roundedDrawable(FIELD, FIELD));
        return input;
    }

    private EditText numberInput(String value) {
        EditText input = input(value);
        input.setText(value);
        input.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL | InputType.TYPE_NUMBER_FLAG_SIGNED);
        return input;
    }

    private LinearLayout labeledInput(String title, EditText input) {
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        TextView label = label(title, 12, MUTED);
        label.setPadding(0, dp(8), 0, dp(4));
        box.addView(label, matchWrap());
        box.addView(input, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(48)));
        return box;
    }

    private Button button(String text, int style) {
        Button button = new Button(this);
        button.setText(text);
        button.setTextSize(15);
        button.setAllCaps(false);
        button.setStateListAnimator(null);
        button.setTypeface(Typeface.create("sans-serif-medium", Typeface.NORMAL));
        button.setMinHeight(dp(48));
        button.setPadding(dp(12), 0, dp(12), 0);
        if (style == STYLE_PRIMARY) {
            button.setTextColor(BG);
            button.setBackground(buttonDrawable(TEXT, Color.rgb(196, 196, 200), TEXT));
        } else if (style == STYLE_DANGER) {
            button.setTextColor(Color.WHITE);
            button.setBackground(buttonDrawable(DANGER, Color.rgb(180, 52, 56), DANGER));
        } else {
            button.setTextColor(TEXT);
            button.setBackground(buttonDrawable(BG, PRESSED, LINE));
        }
        return button;
    }

    private StateListDrawable buttonDrawable(int normal, int pressed, int stroke) {
        StateListDrawable states = new StateListDrawable();
        states.addState(new int[]{android.R.attr.state_pressed}, roundedDrawable(pressed, stroke));
        states.addState(new int[]{}, roundedDrawable(normal, stroke));
        return states;
    }

    private GradientDrawable roundedDrawable(int color, int stroke) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(color);
        drawable.setCornerRadius(dp(6));
        drawable.setStroke(dp(1), stroke);
        return drawable;
    }

    private LinearLayout row() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        return row;
    }

    private LinearLayout.LayoutParams matchWrap() {
        return new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
    }

    private LinearLayout.LayoutParams topMargin(int marginDp) {
        LinearLayout.LayoutParams params = matchWrap();
        params.setMargins(0, dp(marginDp), 0, 0);
        return params;
    }

    private LinearLayout.LayoutParams sizedTopMargin(int width, int height, int marginDp) {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(width, height);
        params.setMargins(0, dp(marginDp), 0, 0);
        return params;
    }

    private LinearLayout.LayoutParams weighted(int leftDp, int rightDp) {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        params.setMargins(dp(leftDp), 0, dp(rightDp), 0);
        return params;
    }

    private LinearLayout.LayoutParams fixedWidth(int widthDp) {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(dp(widthDp), dp(48));
        params.setMargins(dp(8), 0, 0, 0);
        return params;
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }

    // ---------------------------------------------------------------- parsing helpers

    private static String firstNumber(String text) {
        StringBuilder number = new StringBuilder();
        for (int i = 0; i < text.length(); i++) {
            char c = text.charAt(i);
            if ((c >= '0' && c <= '9') || c == '.') {
                number.append(c);
            } else if (number.length() > 0) {
                break;
            }
        }
        return number.length() > 0 ? number.toString() : null;
    }

    private int readInt(EditText input, int fallback) {
        try {
            return Integer.parseInt(input.getText().toString().trim());
        } catch (NumberFormatException ex) {
            return fallback;
        }
    }

    private float readFloat(EditText input, float fallback) {
        try {
            return Float.parseFloat(input.getText().toString().trim().replace(',', '.'));
        } catch (NumberFormatException ex) {
            return fallback;
        }
    }

    private float parseFloatToken(String[] tokens, int index, float fallback) {
        if (index < 0 || index >= tokens.length) {
            return fallback;
        }
        try {
            return Float.parseFloat(tokens[index].trim());
        } catch (NumberFormatException ex) {
            return fallback;
        }
    }

    private int clampInt(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    private float clampFloat(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    private void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) {
            Thread.currentThread().interrupt();
        }
    }

    private static class SimpleWatcher implements TextWatcher {
        private final Runnable onChange;

        SimpleWatcher(Runnable onChange) {
            this.onChange = onChange;
        }

        @Override
        public void beforeTextChanged(CharSequence s, int start, int count, int after) {
        }

        @Override
        public void onTextChanged(CharSequence s, int start, int before, int count) {
        }

        @Override
        public void afterTextChanged(Editable s) {
            onChange.run();
        }
    }
}
