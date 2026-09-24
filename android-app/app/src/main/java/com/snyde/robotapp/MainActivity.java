package com.snyde.robotapp;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.BluetoothSocket;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.StateListDrawable;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;
import android.provider.Settings;
import android.text.InputType;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.HashSet;
import java.util.Set;
import java.util.UUID;

@SuppressWarnings("deprecation")
public class MainActivity extends Activity {
    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final UUID GRUZIK_SERVICE_UUID = UUID.fromString("8f1d0001-8a6b-4dd2-9b52-8ea36a45f001");
    private static final UUID GRUZIK_COMMAND_UUID = UUID.fromString("8f1d0003-8a6b-4dd2-9b52-8ea36a45f001");
    private static final UUID GRUZIK_TEXT_UUID = UUID.fromString("8f1d0004-8a6b-4dd2-9b52-8ea36a45f001");
    private static final UUID CLIENT_CONFIG_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    private static final String PREFS = "robot_presets";
    private static final String KEY_NAMES = "preset_names";
    private static final String KEY_CLEAN_SPEED = "clean.speed";
    private static final String KEY_OPT_STEP = "opt.step";
    private static final String KEY_OPT_SMOOTH = "opt.smooth";
    private static final String KEY_OPT_BASE_SPEED = "opt.base_speed";
    private static final String KEY_OPT_GAIN = "opt.gain";
    private static final String KEY_OPT_MIN_SPEED = "opt.min_speed";
    private static final String KEY_OPT_MAX_SPEED = "opt.max_speed";
    private static final String KEY_MAP_P = "map.p";
    private static final String KEY_MAP_I = "map.i";
    private static final String KEY_MAP_D = "map.d";
    private static final String KEY_MAP_SPEED = "map.speed";
    private static final String KEY_JOYSTICK_SPEED = "joystick.speed";
    private static final String KEY_BT_NAME = "bt.name";
    private static final int BT_NAME_MAX_LEN = 20;
    private static final int REQUEST_BT_CONNECT = 41;
    private static final int TAB_DRIVE = 0;
    private static final int TAB_MAPPING = 1;
    private static final int TAB_JOYSTICK = 2;
    private static final int TAB_ODOMETRY = 3;
    private static final int TAB_DEBUG = 4;

    private static final int BG = Color.rgb(14, 16, 20);
    private static final int SURFACE = Color.rgb(24, 28, 34);
    private static final int SURFACE_2 = Color.rgb(31, 36, 44);
    private static final int SURFACE_PRESSED = Color.rgb(44, 52, 62);
    private static final int STROKE = Color.rgb(58, 66, 78);
    private static final int TEXT = Color.rgb(232, 238, 246);
    private static final int MUTED = Color.rgb(148, 163, 184);
    private static final int ACCENT = Color.rgb(20, 184, 166);
    private static final int ACCENT_PRESSED = Color.rgb(15, 118, 110);
    private static final int GREEN = Color.rgb(34, 197, 94);
    private static final int RED = Color.rgb(239, 68, 68);
    private static final int AMBER = Color.rgb(245, 158, 11);

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final Object writeLock = new Object();
    private final ArrayList<BluetoothDevice> pairedDevices = new ArrayList<>();
    // Only these entries use the ESP32-S3 BLE transport. Every bonded device
    // remains available through the original HC-04/HC-05 Classic SPP route.
    private final HashSet<String> bleDeviceAddresses = new HashSet<>();
    private final ArrayDeque<byte[]> pendingWrites = new ArrayDeque<>();
    private final LinkedHashMap<String, EditText> fields = new LinkedHashMap<>();
    private final ArrayList<String> presetNames = new ArrayList<>();
    private final ArrayList<RoutePoint> recordedRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> optimizedRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> robotOptimizedRoute = new ArrayList<>();
    private final ArrayList<RoutePoint> incomingRoute = new ArrayList<>();
    private final StringBuilder rxLineBuffer = new StringBuilder();

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLeScanner bleScanner;
    private BluetoothGatt bluetoothGatt;
    private BluetoothGattCharacteristic commandCharacteristic;
    private BluetoothGattCharacteristic textCharacteristic;
    private BluetoothSocket socket;
    private OutputStream classicOutputStream;
    private Thread classicReadThread;
    private volatile boolean classicReadThreadRunning;
    private boolean scanRunning;
    private boolean writeInProgress;
    private String connectedDeviceName = "gruzik 5.0";
    private volatile boolean uploadRunning;
    private volatile long lastStartAckMs;

    private Spinner deviceSpinner;
    private Spinner presetSpinner;
    private TextView statusText;
    private TextView logText;
    private TextView voltageText;
    private TextView routeInfoText;
    private TextView transferText;
    private EditText presetNameInput;
    private EditText cleanSpeedInput;

    private LinearLayout drivePage;
    private LinearLayout mappingPage;
    private LinearLayout joystickPage;
    private LinearLayout odometryPage;
    private LinearLayout debugPage;
    private Button driveTabButton;
    private Button mappingTabButton;
    private Button joystickTabButton;
    private Button odometryTabButton;
    private Button debugTabButton;
    private RouteMapView routeMapView;

    private TextView odomXText;
    private TextView odomYText;
    private TextView odomYawText;
    private TextView odomDistanceText;
    private TextView odomSpeedText;
    private TextView odomGyroText;
    private TextView odomFusionText;
    private TextView debugLineSummaryText;
    private TextView debugLineRawText;
    private TextView debugEncoderText;
    private TextView debugImuText;

    private EditText optStepInput;
    private EditText optSmoothInput;
    private EditText optBaseSpeedInput;
    private EditText optGainInput;
    private EditText optMinSpeedInput;
    private EditText optMaxSpeedInput;
    private EditText mapPInput;
    private EditText mapIInput;
    private EditText mapDInput;
    private EditText mapSpeedInput;
    private EditText joystickSpeedInput;
    private EditText btNameInput;
    private TextView btNameStatusText;
    private TextView joystickStatusText;
    private long lastManualSendMs;
    private int lastManualLeft;
    private int lastManualRight;

    private String incomingMapKind;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        buildUi();
        requestBluetoothPermissionIfNeeded();
        loadPresetNames();
        startBleScan();
        if (!presetNames.isEmpty()) {
            loadPreset(presetNames.get(0));
        } else {
            setDefaultValues();
        }
        loadMappingSettings();
        updateRouteUi();
    }

    @Override
    protected void onPause() {
        saveMappingSettings();
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        sendManualStopBlocking();
        sendTireCleanStopBlocking();
        closeConnection();
        super.onDestroy();
    }

    private void buildUi() {
        ScrollView scrollView = new ScrollView(this);
        scrollView.setFillViewport(true);
        scrollView.setBackgroundColor(BG);

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(dp(16), dp(14), dp(16), dp(18));
        scrollView.addView(root, matchWrap());

        TextView title = label("GRUZIK4.0", 26, TEXT);
        title.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        title.setGravity(Gravity.CENTER);
        root.addView(title, matchWrap());

        statusText = label("Disconnected", 14, MUTED);
        statusText.setGravity(Gravity.CENTER);
        root.addView(statusText, matchWrap());

        root.addView(connectionPanel(), matchWrap());
        root.addView(tabRow(), matchWrap());

        drivePage = new LinearLayout(this);
        drivePage.setOrientation(LinearLayout.VERTICAL);
        mappingPage = new LinearLayout(this);
        mappingPage.setOrientation(LinearLayout.VERTICAL);
        joystickPage = new LinearLayout(this);
        joystickPage.setOrientation(LinearLayout.VERTICAL);
        odometryPage = new LinearLayout(this);
        odometryPage.setOrientation(LinearLayout.VERTICAL);
        debugPage = new LinearLayout(this);
        debugPage.setOrientation(LinearLayout.VERTICAL);

        buildDrivePage(drivePage);
        buildMappingPage(mappingPage);
        buildJoystickPage(joystickPage);
        buildOdometryPage(odometryPage);
        buildDebugPage(debugPage);
        root.addView(drivePage, matchWrap());
        root.addView(mappingPage, matchWrap());
        root.addView(joystickPage, matchWrap());
        root.addView(odometryPage, matchWrap());
        root.addView(debugPage, matchWrap());

        addSpacer(root, 8);
        logText = label("", 13, TEXT);
        logText.setMinLines(7);
        logText.setPadding(dp(10), dp(10), dp(10), dp(10));
        logText.setBackground(panelDrawable(SURFACE, STROKE));
        root.addView(logText, matchWrap());

        setContentView(scrollView);
        selectTab(TAB_DRIVE);
    }

    private LinearLayout connectionPanel() {
        LinearLayout panel = panel();
        deviceSpinner = new Spinner(this);
        panel.addView(deviceSpinner, matchWrap());

        LinearLayout connectionRow = row();
        Button refreshButton = button("Refresh");
        Button connectButton = button("Connect");
        Button disconnectButton = button("Disconnect");
        connectionRow.addView(refreshButton, weightWrap());
        connectionRow.addView(connectButton, weightWrap());
        connectionRow.addView(disconnectButton, weightWrap());
        panel.addView(connectionRow);

        refreshButton.setOnClickListener(v -> startBleScan());
        connectButton.setOnClickListener(v -> connectSelectedDevice());
        disconnectButton.setOnClickListener(v -> closeConnection());
        return panel;
    }

    private LinearLayout tabRow() {
        LinearLayout tabs = new LinearLayout(this);
        tabs.setOrientation(LinearLayout.VERTICAL);
        tabs.setPadding(0, dp(8), 0, dp(8));

        LinearLayout topRow = row();
        LinearLayout bottomRow = row();
        driveTabButton = button("Drive");
        mappingTabButton = button("Mapping");
        joystickTabButton = button("Joystick");
        odometryTabButton = button("Odometry");
        debugTabButton = button("Debug");
        topRow.addView(driveTabButton, weightWrap());
        topRow.addView(mappingTabButton, weightWrap());
        topRow.addView(joystickTabButton, weightWrap());
        bottomRow.addView(odometryTabButton, weightWrap());
        bottomRow.addView(debugTabButton, weightWrap());
        tabs.addView(topRow, matchWrap());
        tabs.addView(bottomRow, matchWrap());

        driveTabButton.setOnClickListener(v -> selectTab(TAB_DRIVE));
        mappingTabButton.setOnClickListener(v -> selectTab(TAB_MAPPING));
        joystickTabButton.setOnClickListener(v -> selectTab(TAB_JOYSTICK));
        odometryTabButton.setOnClickListener(v -> selectTab(TAB_ODOMETRY));
        debugTabButton.setOnClickListener(v -> selectTab(TAB_DEBUG));
        return tabs;
    }

    private void buildDrivePage(LinearLayout root) {
        LinearLayout lipoRow = row();
        TextView lipoLabel = blockLabel("LiPo");
        voltageText = blockValue("Voltage");
        lipoRow.addView(lipoLabel, new LinearLayout.LayoutParams(0, dp(58), 1.0f));
        lipoRow.addView(voltageText, new LinearLayout.LayoutParams(0, dp(58), 1.0f));
        root.addView(lipoRow, matchWrap());

        LinearLayout driveRow = row();
        Button normalModeButton = button("Normal mode");
        Button startButton = primaryButton("Start", GREEN);
        Button stopButton = primaryButton("STOP", RED);
        driveRow.addView(normalModeButton, weightWrap());
        driveRow.addView(startButton, weightWrap());
        driveRow.addView(stopButton, weightWrap());
        root.addView(driveRow);

        normalModeButton.setOnClickListener(v -> sendCommand("Mode", "P"));
        startButton.setOnClickListener(v -> startNormalDrive());
        stopButton.setOnClickListener(v -> stopRobot());

        root.addView(tireCleaningPanel(), matchWrap());
        root.addView(presetPanel(), matchWrap());

        LinearLayout fieldsPanel = new LinearLayout(this);
        fieldsPanel.setOrientation(LinearLayout.VERTICAL);
        root.addView(fieldsPanel, matchWrap());

        addField(fieldsPanel, "Turbine_Speed", "Turbine\nSpeed", "0");
        addField(fieldsPanel, "Treshold", "Treshold", "3300");
        addField(fieldsPanel, "Kp", "Kp", "0.015");
        addField(fieldsPanel, "Kd", "Kd", "0.55");
        addField(fieldsPanel, "Base_speed", "Base speed", "125");
        addField(fieldsPanel, "Max_speed", "Max speed", "200");
        addField(fieldsPanel, "Sharp_bend_speed_right", "Sharp Bend\nR", "-75");
        addField(fieldsPanel, "Sharp_bend_speed_left", "Sharp Bend\nL", "120");
        addField(fieldsPanel, "Bend_speed_right", "Base Bend\nR", "-75");
        addField(fieldsPanel, "Bend_speed_left", "Base Bend\nL", "120");
        addField(fieldsPanel, "Turbine_Prep_Time", "Turbine Prep\nTime", "0");
    }

    private LinearLayout tireCleaningPanel() {
        LinearLayout panel = panel();
        panel.addView(sectionTitle("Czyszczenie opon"), matchWrap());
        cleanSpeedInput = smallInput("170");

        LinearLayout controls = row();
        controls.addView(labeledInput("Speed", cleanSpeedInput),
                new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.9f));
        Button cleanButton = primaryButton("Hold to clean", AMBER);
        controls.addView(cleanButton, new LinearLayout.LayoutParams(0, dp(76), 1.1f));
        panel.addView(controls, matchWrap());

        cleanButton.setOnTouchListener((view, event) -> {
            int action = event.getActionMasked();
            if (action == MotionEvent.ACTION_DOWN) {
                view.setAlpha(0.72f);
                sendTireCleanStart();
                return true;
            } else if (action == MotionEvent.ACTION_UP || action == MotionEvent.ACTION_CANCEL) {
                view.setAlpha(1.0f);
                sendTireCleanStop();
                return true;
            }
            return true;
        });
        return panel;
    }

    private LinearLayout presetPanel() {
        LinearLayout presetPanel = panel();
        presetNameInput = input("Preset name");
        presetPanel.addView(presetNameInput, matchWrap());

        presetSpinner = new Spinner(this);
        presetPanel.addView(presetSpinner, matchWrap());
        presetSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (position >= 0 && position < presetNames.size()) {
                    loadPreset(presetNames.get(position));
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });

        LinearLayout presetRow = row();
        Button savePresetButton = button("Save");
        Button deletePresetButton = button("Delete");
        Button sendPresetButton = button("Send values");
        presetRow.addView(savePresetButton, weightWrap());
        presetRow.addView(deletePresetButton, weightWrap());
        presetRow.addView(sendPresetButton, weightWrap());
        presetPanel.addView(presetRow);

        savePresetButton.setOnClickListener(v -> saveCurrentPreset());
        deletePresetButton.setOnClickListener(v -> deleteSelectedPreset());
        sendPresetButton.setOnClickListener(v -> sendPresetValuesAsync());
        return presetPanel;
    }

    private void buildMappingPage(LinearLayout root) {
        routeInfoText = label("", 14, MUTED);
        transferText = label("BT map transfer idle", 13, MUTED);
        root.addView(routeInfoText, matchWrap());
        root.addView(transferText, matchWrap());

        routeMapView = new RouteMapView(this);
        LinearLayout.LayoutParams mapParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(300));
        mapParams.setMargins(0, dp(6), 0, dp(8));
        root.addView(routeMapView, mapParams);

        LinearLayout modePanel = panel();
        TextView modeTitle = sectionTitle("Tryby robota");
        modePanel.addView(modeTitle, matchWrap());
        LinearLayout modeRow = row();
        Button normalButton = button("Normal");
        Button mappingButton = button("Mapping");
        Button playbackButton = button("Playback");
        modeRow.addView(normalButton, weightWrap());
        modeRow.addView(mappingButton, weightWrap());
        modeRow.addView(playbackButton, weightWrap());
        modePanel.addView(modeRow);

        LinearLayout actionRow = row();
        Button startMappingButton = primaryButton("Start map", ACCENT);
        Button playbackStartButton = primaryButton("Run map", GREEN);
        Button stopButton = primaryButton("STOP", RED);
        actionRow.addView(startMappingButton, weightWrap());
        actionRow.addView(playbackStartButton, weightWrap());
        actionRow.addView(stopButton, weightWrap());
        modePanel.addView(actionRow);
        root.addView(modePanel, matchWrap());

        normalButton.setOnClickListener(v -> sendCommand("Mode", "P"));
        mappingButton.setOnClickListener(v -> sendCommand("Mode", "M"));
        playbackButton.setOnClickListener(v -> sendCommand("Mode", "U"));
        startMappingButton.setOnClickListener(v -> startMappingRun());
        playbackStartButton.setOnClickListener(v -> startPlaybackRun());
        stopButton.setOnClickListener(v -> stopRobot());

        root.addView(mapTransferPanel(), matchWrap());
        root.addView(optimizationPanel(), matchWrap());
        root.addView(mapControllerPanel(), matchWrap());
    }

    private LinearLayout mapTransferPanel() {
        LinearLayout panel = panel();
        panel.addView(sectionTitle("Mapa z robota"), matchWrap());
        LinearLayout row1 = row();
        Button downloadRecorded = button("Get saved");
        Button downloadRobotMap = button("Get map.txt");
        Button clearButton = button("Clear view");
        row1.addView(downloadRecorded, weightWrap());
        row1.addView(downloadRobotMap, weightWrap());
        row1.addView(clearButton, weightWrap());
        panel.addView(row1);

        downloadRecorded.setOnClickListener(v -> sendCommand("MapDump", "recorded"));
        downloadRobotMap.setOnClickListener(v -> sendCommand("MapDump", "optimized"));
        clearButton.setOnClickListener(v -> {
            recordedRoute.clear();
            optimizedRoute.clear();
            robotOptimizedRoute.clear();
            updateRouteUi();
        });
        return panel;
    }

    private LinearLayout optimizationPanel() {
        LinearLayout panel = panel();
        panel.addView(sectionTitle("Optymalizacja"), matchWrap());

        optStepInput = smallInput("80");
        optSmoothInput = smallInput("5");
        optBaseSpeedInput = smallInput("40");
        optGainInput = smallInput("60");
        optMinSpeedInput = smallInput("35");
        optMaxSpeedInput = smallInput("95");

        panel.addView(twoInputs("Point step", optStepInput, "Smoothing", optSmoothInput));
        panel.addView(twoInputs("Base speed", optBaseSpeedInput, "Gain", optGainInput));
        panel.addView(twoInputs("Min speed", optMinSpeedInput, "Max speed", optMaxSpeedInput));

        LinearLayout row = row();
        Button optimizeButton = primaryButton("Optimize", ACCENT);
        Button uploadButton = primaryButton("Upload map.txt", AMBER);
        Button saveButton = button("Save settings");
        row.addView(optimizeButton, weightWrap());
        row.addView(uploadButton, weightWrap());
        row.addView(saveButton, weightWrap());
        panel.addView(row);

        optimizeButton.setOnClickListener(v -> optimizeRecordedRoute());
        uploadButton.setOnClickListener(v -> uploadOptimizedRoute());
        saveButton.setOnClickListener(v -> {
            saveMappingSettings();
            toast("Mapping settings saved");
        });
        return panel;
    }

    private LinearLayout mapControllerPanel() {
        LinearLayout panel = panel();
        panel.addView(sectionTitle("Kontroler odtwarzania"), matchWrap());
        mapPInput = smallInput("90");
        mapIInput = smallInput("0");
        mapDInput = smallInput("10");
        mapSpeedInput = smallInput("80");
        panel.addView(twoInputs("MapP", mapPInput, "MapI", mapIInput));
        panel.addView(twoInputs("MapD", mapDInput, "Default speed", mapSpeedInput));
        LinearLayout row = row();
        Button sendButton = button("Send map controller");
        Button saveButton = button("Save settings");
        row.addView(sendButton, weightWrap());
        row.addView(saveButton, weightWrap());
        panel.addView(row, matchWrap());
        sendButton.setOnClickListener(v -> new Thread(this::sendMappingControllerValuesBlocking).start());
        saveButton.setOnClickListener(v -> {
            saveMappingSettings();
            toast("Mapping settings saved");
        });
        return panel;
    }

    private void buildJoystickPage(LinearLayout root) {
        LinearLayout panel = panel();
        panel.addView(sectionTitle("Joystick"), matchWrap());

        joystickSpeedInput = smallInput("85");
        joystickStatusText = telemetryText("L 0   R 0");
        Button stopButton = primaryButton("STOP manual", RED);

        LinearLayout settingsRow = row();
        settingsRow.addView(labeledInput("Max PWM", joystickSpeedInput),
                new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1.0f));
        settingsRow.addView(stopButton, new LinearLayout.LayoutParams(0, dp(76), 1.0f));
        panel.addView(settingsRow, matchWrap());

        JoystickView joystick = new JoystickView(this);
        LinearLayout.LayoutParams joystickParams = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(290));
        joystickParams.setMargins(0, dp(8), 0, dp(8));
        panel.addView(joystick, joystickParams);
        panel.addView(joystickStatusText, matchWrap());
        root.addView(panel, matchWrap());

        joystick.setListener((forward, turn, active) -> sendManualDrive(forward, turn, !active));
        stopButton.setOnClickListener(v -> sendManualStop());
    }

    private void buildOdometryPage(LinearLayout root) {
        LinearLayout controlPanel = panel();
        controlPanel.addView(sectionTitle("Odometry"), matchWrap());
        LinearLayout row = row();
        Button startButton = primaryButton("Start odom", ACCENT);
        Button debugButton = button("Debug tab");
        Button stopButton = primaryButton("Stop stream", RED);
        row.addView(startButton, weightWrap());
        row.addView(debugButton, weightWrap());
        row.addView(stopButton, weightWrap());
        controlPanel.addView(row);
        root.addView(controlPanel, matchWrap());

        odomXText = blockValue("0.000 m");
        odomYText = blockValue("0.000 m");
        odomYawText = blockValue("0.0 deg");
        odomDistanceText = blockValue("0.000 m");
        odomSpeedText = telemetryText("L 0.000 / R 0.000 m/s");
        odomGyroText = blockValue("0.00 dps");
        odomFusionText = telemetryText("enc 0.0 deg / gyro 0.0 deg");

        root.addView(twoReadouts("X", odomXText, "Y", odomYText), matchWrap());
        root.addView(twoReadouts("Yaw", odomYawText, "Distance", odomDistanceText), matchWrap());
        root.addView(readoutPanel("Wheel speed", odomSpeedText), matchWrap());
        root.addView(twoReadouts("Gyro Z", odomGyroText, "Fusion", odomFusionText), matchWrap());

        startButton.setOnClickListener(v -> sendCommand("Telemetry", "odom"));
        debugButton.setOnClickListener(v -> selectTab(TAB_DEBUG));
        stopButton.setOnClickListener(v -> sendCommand("Telemetry", "off"));
    }

    private LinearLayout bluetoothNamePanel() {
        LinearLayout panel = panel();
        panel.addView(sectionTitle("Bluetooth name"), matchWrap());
        btNameInput = input("GRUZIK4.0");
        btNameInput.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_FLAG_CAP_CHARACTERS);
        btNameStatusText = telemetryText("Classic: rename HC module. ESP32-S3 BLE: gruzik 5.0");
        LinearLayout inputRow = row();
        inputRow.addView(labeledInput("Name", btNameInput),
                new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1.15f));
        Button saveButton = primaryButton("Save for KEY", ACCENT);
        inputRow.addView(saveButton, new LinearLayout.LayoutParams(0, dp(76), 0.85f));
        panel.addView(inputRow, matchWrap());
        LinearLayout actionRow = row();
        Button tryNowButton = button("Try AT now");
        Button clearButton = button("Clear status");
        actionRow.addView(tryNowButton, weightWrap());
        actionRow.addView(clearButton, weightWrap());
        panel.addView(actionRow, matchWrap());
        panel.addView(btNameStatusText, matchWrap());
        saveButton.setOnClickListener(v -> sendBluetoothName(false));
        tryNowButton.setOnClickListener(v -> confirmBluetoothNameNow());
        clearButton.setOnClickListener(v -> btNameStatusText.setText("Ready"));
        return panel;
    }

    private void buildDebugPage(LinearLayout root) {
        LinearLayout controlPanel = panel();
        controlPanel.addView(sectionTitle("Debug"), matchWrap());
        LinearLayout row = row();
        Button debugStreamButton = primaryButton("Debug stream", ACCENT);
        Button odomStreamButton = button("Odom stream");
        Button stopButton = primaryButton("Stop stream", RED);
        row.addView(debugStreamButton, weightWrap());
        row.addView(odomStreamButton, weightWrap());
        row.addView(stopButton, weightWrap());
        controlPanel.addView(row);
        root.addView(controlPanel, matchWrap());
        root.addView(bluetoothNamePanel(), matchWrap());

        debugLineSummaryText = telemetryText("Line sensors idle");
        debugLineRawText = telemetryText("S01..S12: waiting");
        debugEncoderText = telemetryText("Encoders waiting");
        debugImuText = telemetryText("IMU waiting");

        root.addView(readoutPanel("Line sensor", debugLineSummaryText), matchWrap());
        root.addView(readoutPanel("Line raw ADC", debugLineRawText), matchWrap());
        root.addView(readoutPanel("Encoders", debugEncoderText), matchWrap());
        root.addView(readoutPanel("IMU", debugImuText), matchWrap());

        debugStreamButton.setOnClickListener(v -> sendCommand("Telemetry", "debug"));
        odomStreamButton.setOnClickListener(v -> sendCommand("Telemetry", "odom"));
        stopButton.setOnClickListener(v -> sendCommand("Telemetry", "off"));
    }

    private void addField(LinearLayout parent, String key, String displayName, String defaultValue) {
        LinearLayout row = row();
        row.setPadding(0, dp(4), 0, dp(4));
        TextView label = blockLabel(displayName);
        EditText input = input(defaultValue);
        input.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL | InputType.TYPE_NUMBER_FLAG_SIGNED);
        Button sendButton = button("Send");
        sendButton.setOnClickListener(v -> sendCommand(key, input.getText().toString().trim()));
        row.addView(label, new LinearLayout.LayoutParams(0, dp(62), 0.95f));
        row.addView(input, new LinearLayout.LayoutParams(0, dp(62), 1.45f));
        row.addView(sendButton, new LinearLayout.LayoutParams(0, dp(62), 0.8f));
        parent.addView(row);
        fields.put(key, input);
    }

    private void selectTab(int tab) {
        drivePage.setVisibility(tab == TAB_DRIVE ? View.VISIBLE : View.GONE);
        mappingPage.setVisibility(tab == TAB_MAPPING ? View.VISIBLE : View.GONE);
        joystickPage.setVisibility(tab == TAB_JOYSTICK ? View.VISIBLE : View.GONE);
        odometryPage.setVisibility(tab == TAB_ODOMETRY ? View.VISIBLE : View.GONE);
        debugPage.setVisibility(tab == TAB_DEBUG ? View.VISIBLE : View.GONE);
        driveTabButton.setBackground(buttonDrawable(tab == TAB_DRIVE ? ACCENT : SURFACE_2, tab == TAB_DRIVE ? ACCENT_PRESSED : SURFACE_PRESSED));
        mappingTabButton.setBackground(buttonDrawable(tab == TAB_MAPPING ? ACCENT : SURFACE_2, tab == TAB_MAPPING ? ACCENT_PRESSED : SURFACE_PRESSED));
        joystickTabButton.setBackground(buttonDrawable(tab == TAB_JOYSTICK ? ACCENT : SURFACE_2, tab == TAB_JOYSTICK ? ACCENT_PRESSED : SURFACE_PRESSED));
        odometryTabButton.setBackground(buttonDrawable(tab == TAB_ODOMETRY ? ACCENT : SURFACE_2, tab == TAB_ODOMETRY ? ACCENT_PRESSED : SURFACE_PRESSED));
        debugTabButton.setBackground(buttonDrawable(tab == TAB_DEBUG ? ACCENT : SURFACE_2, tab == TAB_DEBUG ? ACCENT_PRESSED : SURFACE_PRESSED));
    }

    private void requestBluetoothPermissionIfNeeded() {
        ArrayList<String> permissions = missingBluetoothPermissions();
        if (!permissions.isEmpty()) {
            requestPermissions(permissions.toArray(new String[0]), REQUEST_BT_CONNECT);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQUEST_BT_CONNECT) {
            startBleScan();
        }
    }

    private boolean hasBtPermission() {
        return missingBluetoothPermissions().isEmpty();
    }

    private ArrayList<String> missingBluetoothPermissions() {
        ArrayList<String> permissions = new ArrayList<>();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            if (checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.BLUETOOTH_CONNECT);
            }
            if (checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.BLUETOOTH_SCAN);
            }
        } else if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.ACCESS_FINE_LOCATION);
            }
        }
        return permissions;
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            BluetoothDevice device = result.getDevice();
            String advertisedName = null;
            boolean serviceMatches = false;
            if (result.getScanRecord() != null) {
                advertisedName = result.getScanRecord().getDeviceName();
                List<ParcelUuid> services = result.getScanRecord().getServiceUuids();
                if (services != null) {
                    for (ParcelUuid service : services) {
                        if (GRUZIK_SERVICE_UUID.equals(service.getUuid())) {
                            serviceMatches = true;
                            break;
                        }
                    }
                }
            }
            try {
                if (advertisedName == null) {
                    advertisedName = device.getName();
                }
            } catch (SecurityException ignored) {
            }
            boolean nameMatches = advertisedName != null &&
                    advertisedName.toLowerCase(Locale.ROOT).contains("gruzik");
            if (!serviceMatches && !nameMatches) {
                return;
            }

            for (BluetoothDevice existing : pairedDevices) {
                if (existing.getAddress().equals(device.getAddress())) {
                    bleDeviceAddresses.add(device.getAddress());
                    return;
                }
            }
            pairedDevices.add(device);
            bleDeviceAddresses.add(device.getAddress());
            refreshDeviceSpinner();
        }

        @Override
        public void onScanFailed(int errorCode) {
            scanRunning = false;
            setStatus("BLE scan failed: " + errorCode);
        }
    };

    private void startBleScan() {
        pairedDevices.clear();
        bleDeviceAddresses.clear();
        ArrayList<String> names = new ArrayList<>();

        if (bluetoothAdapter == null) {
            names.add("Bluetooth not available");
            setDeviceAdapter(names);
            return;
        }
        if (!hasBtPermission()) {
            names.add("Bluetooth permission needed");
            setDeviceAdapter(names);
            setStatus("Bluetooth permission needed");
            return;
        }
        if (!bluetoothAdapter.isEnabled()) {
            names.add("Enable Bluetooth first");
            setDeviceAdapter(names);
            return;
        }

        try {
            // Preserve the legacy device list. They are bonded and continue to
            // connect via Classic SPP exactly as before.
            Set<BluetoothDevice> bondedDevices = bluetoothAdapter.getBondedDevices();
            for (BluetoothDevice device : bondedDevices) {
                pairedDevices.add(device);
            }
            if (scanRunning && bleScanner != null) {
                bleScanner.stopScan(scanCallback);
            }
            bleScanner = bluetoothAdapter.getBluetoothLeScanner();
            if (bleScanner == null) {
                names.add("BLE scanner unavailable");
                setDeviceAdapter(names);
                return;
            }
            refreshDeviceSpinner();
            setStatus("Scanning BLE...");
            scanRunning = true;
            bleScanner.startScan(scanCallback);
            mainHandler.postDelayed(() -> {
                if (!scanRunning || bleScanner == null) {
                    return;
                }
                try {
                    bleScanner.stopScan(scanCallback);
                } catch (SecurityException ignored) {
                }
                scanRunning = false;
                refreshDeviceSpinner();
                if (bleDeviceAddresses.isEmpty()) {
                    setStatus("gruzik 5.0 not found - tap Refresh");
                }
            }, 6000);
        } catch (SecurityException ex) {
            names.add("Bluetooth permission blocked");
            setDeviceAdapter(names);
            setStatus("Bluetooth permission blocked by Android");
            showPermissionHelpIfNeeded();
        }
    }

    private void refreshDeviceSpinner() {
        ArrayList<String> names = new ArrayList<>();
        for (BluetoothDevice device : pairedDevices) {
            String name = bleDeviceAddresses.contains(device.getAddress()) ? "gruzik 5.0" : "Unknown";
            try {
                if (device.getName() != null) {
                    name = device.getName();
                }
            } catch (SecurityException ignored) {
            }
            String transport = bleDeviceAddresses.contains(device.getAddress()) ? "BLE" : "Classic";
            names.add(name + " [" + transport + "]  " + device.getAddress());
        }
        if (names.isEmpty()) {
            names.add(scanRunning ? "Scanning for gruzik 5.0..." : "No paired devices / gruzik 5.0 not found");
        }
        setDeviceAdapter(names);
    }

    private void showPermissionHelpIfNeeded() {
        if (hasBtPermission()) {
            return;
        }

        new AlertDialog.Builder(this)
                .setTitle("Bluetooth permission needed")
                .setMessage("Android blocks Bluetooth access. Open app settings and allow Nearby devices/Bluetooth.")
                .setPositiveButton("App settings", (dialog, which) -> {
                    Intent intent = new Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
                    intent.setData(Uri.fromParts("package", getPackageName(), null));
                    startActivity(intent);
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void setDeviceAdapter(ArrayList<String> names) {
        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, names);
        deviceSpinner.setAdapter(adapter);
    }

    private void connectSelectedDevice() {
        if (!hasBtPermission()) {
            requestBluetoothPermissionIfNeeded();
            showPermissionHelpIfNeeded();
            return;
        }
        int index = deviceSpinner.getSelectedItemPosition();
        if (index < 0 || index >= pairedDevices.size()) {
            toast("Tap Refresh and select gruzik 5.0");
            return;
        }

        BluetoothDevice device = pairedDevices.get(index);
        setStatus("Connecting...");
        if (!bleDeviceAddresses.contains(device.getAddress())) {
            connectClassicDevice(device);
            return;
        }
        try {
            if (scanRunning && bleScanner != null) {
                bleScanner.stopScan(scanCallback);
                scanRunning = false;
            }
            closeConnection();
            connectedDeviceName = device.getName() == null ? "gruzik 5.0" : device.getName();
            bluetoothGatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
        } catch (SecurityException ex) {
            setStatus("Disconnected");
            showError("BLE connection failed", ex.getMessage());
        }
    }

    private void connectClassicDevice(BluetoothDevice device) {
        new Thread(() -> {
            try {
                closeConnection();
                BluetoothSocket newSocket = device.createRfcommSocketToServiceRecord(SPP_UUID);
                bluetoothAdapter.cancelDiscovery();
                newSocket.connect();
                socket = newSocket;
                classicOutputStream = newSocket.getOutputStream();
                startClassicReadThread(newSocket.getInputStream());
                String name = device.getName() == null ? device.getAddress() : device.getName();
                mainHandler.post(() -> setStatus("Connected (Classic): " + name));
            } catch (IOException | SecurityException ex) {
                mainHandler.post(() -> showError("Bluetooth connection failed", ex.getMessage()));
                closeConnection();
            }
        }, "gruzik-classic-connect").start();
    }

    private void startClassicReadThread(InputStream inputStream) {
        classicReadThreadRunning = false;
        if (classicReadThread != null) {
            classicReadThread.interrupt();
        }
        classicReadThreadRunning = true;
        classicReadThread = new Thread(() -> {
            byte[] buffer = new byte[256];
            while (classicReadThreadRunning) {
                try {
                    int len = inputStream.read(buffer);
                    if (len > 0) {
                        String text = new String(buffer, 0, len, StandardCharsets.UTF_8);
                        mainHandler.post(() -> handleIncomingText(text));
                    }
                } catch (IOException ex) {
                    if (classicReadThreadRunning) {
                        mainHandler.post(() -> setStatus("Disconnected"));
                    }
                    return;
                }
            }
        }, "gruzik-classic-read");
        classicReadThread.start();
    }

    private void closeConnection() {
        classicReadThreadRunning = false;
        try {
            if (socket != null) {
                socket.close();
            }
        } catch (IOException ignored) {
        }
        socket = null;
        classicOutputStream = null;
        BluetoothGatt gatt = bluetoothGatt;
        bluetoothGatt = null;
        commandCharacteristic = null;
        textCharacteristic = null;
        synchronized (writeLock) {
            pendingWrites.clear();
            writeInProgress = false;
        }
        try {
            if (gatt != null) {
                gatt.disconnect();
                gatt.close();
            }
        } catch (SecurityException ignored) {
        }
        mainHandler.post(() -> setStatus("Disconnected"));
    }

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            if (status == BluetoothGatt.GATT_SUCCESS && newState == BluetoothProfile.STATE_CONNECTED) {
                bluetoothGatt = gatt;
                mainHandler.post(() -> setStatus("Discovering gruzik 5.0..."));
                try {
                    if (!gatt.requestMtu(185)) {
                        gatt.discoverServices();
                    }
                } catch (SecurityException ex) {
                    mainHandler.post(() -> showError("BLE service discovery failed", ex.getMessage()));
                }
                return;
            }
            if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                if (bluetoothGatt == gatt) {
                    bluetoothGatt = null;
                    commandCharacteristic = null;
                    textCharacteristic = null;
                }
                synchronized (writeLock) {
                    pendingWrites.clear();
                    writeInProgress = false;
                }
                try {
                    gatt.close();
                } catch (SecurityException ignored) {
                }
                mainHandler.post(() -> setStatus("Disconnected"));
            }
        }

        @Override
        public void onMtuChanged(BluetoothGatt gatt, int mtu, int status) {
            try {
                gatt.discoverServices();
            } catch (SecurityException ex) {
                mainHandler.post(() -> showError("BLE service discovery failed", ex.getMessage()));
            }
        }

        @Override
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                mainHandler.post(() -> showError("BLE error", "Cannot discover robot service"));
                return;
            }
            BluetoothGattService service = gatt.getService(GRUZIK_SERVICE_UUID);
            if (service == null) {
                mainHandler.post(() -> showError("BLE error", "GRUZIK5 service not found"));
                return;
            }
            commandCharacteristic = service.getCharacteristic(GRUZIK_COMMAND_UUID);
            textCharacteristic = service.getCharacteristic(GRUZIK_TEXT_UUID);
            if (commandCharacteristic == null || textCharacteristic == null) {
                mainHandler.post(() -> showError("BLE error", "GRUZIK5 characteristics not found"));
                return;
            }
            try {
                gatt.setCharacteristicNotification(textCharacteristic, true);
                BluetoothGattDescriptor descriptor = textCharacteristic.getDescriptor(CLIENT_CONFIG_UUID);
                if (descriptor != null) {
                    descriptor.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                    gatt.writeDescriptor(descriptor);
                } else {
                    mainHandler.post(() -> setStatus("Connected: " + connectedDeviceName));
                }
            } catch (SecurityException ex) {
                mainHandler.post(() -> showError("BLE notification failed", ex.getMessage()));
            }
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
            if (CLIENT_CONFIG_UUID.equals(descriptor.getUuid()) && status == BluetoothGatt.GATT_SUCCESS) {
                mainHandler.post(() -> {
                    setStatus("Connected: " + connectedDeviceName);
                    // GRUZIK5 uses notifications instead of the legacy UART
                    // stream. Start its diagnostic stream immediately so the
                    // voltage, line sensors and IMU are visible on connect.
                    sendLine("Telemetry=debug\n", false);
                });
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
            if (GRUZIK_TEXT_UUID.equals(characteristic.getUuid())) {
                String text = new String(characteristic.getValue(), StandardCharsets.UTF_8);
                mainHandler.post(() -> handleIncomingText(text));
            }
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, byte[] value) {
            if (GRUZIK_TEXT_UUID.equals(characteristic.getUuid())) {
                String text = new String(value, StandardCharsets.UTF_8);
                mainHandler.post(() -> handleIncomingText(text));
            }
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
            synchronized (writeLock) {
                if (!pendingWrites.isEmpty()) {
                    pendingWrites.removeFirst();
                }
                writeInProgress = false;
            }
            mainHandler.post(MainActivity.this::writeNextBleCommand);
        }
    };

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
        if (line.startsWith("ODOM,")) {
            parseOdomTelemetry(line);
            return;
        }

        if (line.startsWith("DBG,")) {
            parseDebugTelemetry(line);
            return;
        }

        if (line.startsWith("TELEMETRY,")) {
            appendLog("< " + line + "\n");
            return;
        }

        if (line.startsWith("BT_NAME_")) {
            handleBluetoothNameStatus(line);
            appendLog("< " + line + "\n");
            return;
        }

        if (line.startsWith("STARTING,") || "Start".equals(line)) {
            lastStartAckMs = System.currentTimeMillis();
            if (transferText != null && line.startsWith("STARTING,")) {
                transferText.setText(line);
            }
            appendLog("< " + line + "\n");
            return;
        }

        if (line.startsWith("MAP_ERROR") || line.startsWith("SD_ERROR")) {
            if (transferText != null) {
                if ((line.startsWith("MAP_ERROR,open_read,map.txt,4")) ||
                        (line.startsWith("MAP_ERROR,open_playback,map.txt,4"))) {
                    robotOptimizedRoute.clear();
                    updateRouteUi();
                    transferText.setText("map.txt missing on SD - use Upload or Run map");
                } else {
                    transferText.setText(line);
                }
            }
            appendLog("< " + line + "\n");
            return;
        }

        if (line.startsWith("MAP_AUTO_CLOSED")) {
            if (transferText != null) {
                transferText.setText("Mapping auto-closed near start - Get saved");
            }
            appendLog("< " + line + "\n");
            return;
        }

        if (line.startsWith("MAP_BEGIN,")) {
            incomingMapKind = line.substring("MAP_BEGIN,".length()).trim();
            incomingRoute.clear();
            transferText.setText("Receiving " + incomingMapKind + "...");
            appendLog("< " + line + "\n");
            return;
        }

        if (line.startsWith("MAP,") && incomingMapKind != null) {
            RoutePoint point = parseRoutePoint(line.substring(4), incomingMapKind);
            if (point != null) {
                incomingRoute.add(point);
                if (incomingRoute.size() % 50 == 0) {
                    transferText.setText("Receiving " + incomingMapKind + ": " + incomingRoute.size() + " pts");
                }
            }
            return;
        }

        if (line.startsWith("MAP_END") && incomingMapKind != null) {
            boolean autoOptimized = false;
            if ("recorded".equals(incomingMapKind)) {
                recordedRoute.clear();
                recordedRoute.addAll(incomingRoute);
                optimizedRoute.clear();
                autoOptimized = optimizeRecordedRoute(false);
            } else {
                robotOptimizedRoute.clear();
                robotOptimizedRoute.addAll(incomingRoute);
            }
            String message = "Received " + incomingMapKind + ": " + incomingRoute.size() + " pts";
            if (autoOptimized) {
                message += " / optimized: " + optimizedRoute.size() + " pts";
            }
            transferText.setText(message);
            appendLog("< " + line + "\n");
            incomingRoute.clear();
            incomingMapKind = null;
            updateRouteUi();
            return;
        }

        appendLog("< " + line + "\n");
        updateVoltageFromRobotText(line);
        if (line.startsWith("UPLOAD_")) {
            transferText.setText(line);
        }
    }

    private void parseOdomTelemetry(String line) {
        String[] tokens = line.split(",");
        if (tokens.length < 10 || odomXText == null) {
            return;
        }

        float x = parseFloatToken(tokens, 1, 0.0f);
        float y = parseFloatToken(tokens, 2, 0.0f);
        float yaw = parseFloatToken(tokens, 3, 0.0f);
        float distance = parseFloatToken(tokens, 4, 0.0f);
        float leftSpeed = parseFloatToken(tokens, 5, 0.0f);
        float rightSpeed = parseFloatToken(tokens, 6, 0.0f);
        float gyroZ = parseFloatToken(tokens, 7, 0.0f);
        float encoderYaw = parseFloatToken(tokens, 8, 0.0f);
        float gyroYaw = parseFloatToken(tokens, 9, 0.0f);

        odomXText.setText(String.format(Locale.US, "%.3f m", x));
        odomYText.setText(String.format(Locale.US, "%.3f m", y));
        odomYawText.setText(String.format(Locale.US, "%.1f deg", yaw));
        odomDistanceText.setText(String.format(Locale.US, "%.3f m", distance));
        odomSpeedText.setText(String.format(Locale.US, "L %.3f / R %.3f m/s", leftSpeed, rightSpeed));
        odomGyroText.setText(String.format(Locale.US, "%.2f dps", gyroZ));
        odomFusionText.setText(String.format(Locale.US, "enc %.1f deg / gyro %.1f deg", encoderYaw, gyroYaw));
    }

    private void parseDebugTelemetry(String line) {
        String[] tokens = line.split(",");
        if (tokens.length < 14 || debugLineSummaryText == null) {
            return;
        }

        float sensorPosition = parseFloatToken(tokens, 1, 0.0f);
        int activeCount = Math.round(parseFloatToken(tokens, 2, 0.0f));
        int lastEnd = Math.round(parseFloatToken(tokens, 3, 0.0f));
        int leftDelta = Math.round(parseFloatToken(tokens, 4, 0.0f));
        int rightDelta = Math.round(parseFloatToken(tokens, 5, 0.0f));
        float leftRpm = parseFloatToken(tokens, 6, 0.0f);
        float rightRpm = parseFloatToken(tokens, 7, 0.0f);
        float leftDistance = parseFloatToken(tokens, 8, 0.0f);
        float rightDistance = parseFloatToken(tokens, 9, 0.0f);
        float gyroRaw = parseFloatToken(tokens, 10, 0.0f);
        float gyroBias = parseFloatToken(tokens, 11, 0.0f);
        float gyroZ = parseFloatToken(tokens, 12, 0.0f);

        debugLineSummaryText.setText(String.format(Locale.US,
                "pos %.0f   active %d   last %s",
                sensorPosition, activeCount, lastEnd == 1 ? "right" : "left"));

        StringBuilder sensorBuilder = new StringBuilder();
        for (int i = 13; i < tokens.length; i++) {
            if (sensorBuilder.length() > 0) {
                sensorBuilder.append("  ");
            }
            sensorBuilder.append(String.format(Locale.US, "S%02d=%s", i - 13, tokens[i].trim()));
            if (((i - 12) % 4) == 0) {
                sensorBuilder.append("\n");
            }
        }
        debugLineRawText.setText(sensorBuilder.toString());

        debugEncoderText.setText(String.format(Locale.US,
                "delta L %d / R %d cnt\nrpm L %.1f / R %.1f\ndist L %.4f / R %.4f m",
                leftDelta, rightDelta, leftRpm, rightRpm, leftDistance, rightDistance));
        debugImuText.setText(String.format(Locale.US,
                "gyro raw %.2f dps\nbias %.2f dps\ngyro z %.2f dps",
                gyroRaw, gyroBias, gyroZ));
    }

    private RoutePoint parseRoutePoint(String value, String kind) {
        String[] tokens = value.trim().split("[,\\s]+");
        ArrayList<Float> numbers = new ArrayList<>();
        for (String token : tokens) {
            if (token == null || token.isEmpty()) {
                continue;
            }
            try {
                numbers.add(Float.parseFloat(token.replace(',', '.')));
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

    private void startNormalDrive() {
        new Thread(() -> {
            long startRequestMs = System.currentTimeMillis();
            sendLine("Telemetry=off\n", true);
            sleepMs(40);
            sendPresetValuesBlocking();
            sleepMs(120);
            sendLine("StartNormal=1\n", true);
            fallbackStartIfNeeded(startRequestMs, "P");
        }).start();
    }

    private void startMappingRun() {
        saveMappingSettings();
        new Thread(() -> {
            mainHandler.post(() -> transferText.setText("Starting mapping..."));
            long startRequestMs = System.currentTimeMillis();
            sendLine("Telemetry=off\n", true);
            sleepMs(40);
            sendPresetValuesBlocking();
            sleepMs(120);
            sendLine("StartMapping=1\n", true);
            fallbackStartIfNeeded(startRequestMs, "M");
        }).start();
    }

    private void startPlaybackRun() {
        if (optimizedRoute.isEmpty() && !optimizeRecordedRoute(true)) {
            return;
        }
        saveMappingSettings();

        new Thread(() -> {
            mainHandler.post(() -> transferText.setText("Starting playback..."));
            sendLine("Telemetry=off\n", true);
            sleepMs(40);
            sendMappingControllerValuesBlocking();
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
        new Thread(() -> {
            sendManualStopBlocking();
            sleepMs(25);
            sendTireCleanStopBlocking();
            sleepMs(25);
            sendLine("Telemetry=off\n", true);
            sleepMs(40);
            sendLine("Mode=N\n", true);
        }).start();
    }

    private void sendTireCleanStart() {
        saveMappingSettings();
        new Thread(() -> {
            String speed = cleanSpeedInput.getText().toString().trim();
            if (speed.isEmpty()) {
                speed = "170";
            }
            sendLine("CleanSpeed=" + speed + "\n", true);
            sleepMs(25);
            sendLine("Clean=1\n", true);
        }).start();
    }

    private void sendTireCleanStop() {
        new Thread(this::sendTireCleanStopBlocking).start();
    }

    private void sendTireCleanStopBlocking() {
        sendLine("Clean=0\n", true);
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
        String command = String.format(Locale.US, "Manual=%d,%d\n", left, right);
        new Thread(() -> sendLine(command, false)).start();
    }

    private void sendManualStop() {
        new Thread(this::sendManualStopBlocking).start();
    }

    private void sendManualStopBlocking() {
        lastManualLeft = 0;
        lastManualRight = 0;
        lastManualSendMs = 0L;
        if (joystickStatusText != null) {
            mainHandler.post(() -> joystickStatusText.setText("L 0   R 0"));
        }
        sendLine("Manual=0,0\n", true);
    }

    private void sendPresetValuesAsync() {
        new Thread(this::sendPresetValuesBlocking).start();
    }

    private void sendPresetValuesBlocking() {
        for (Map.Entry<String, EditText> entry : fields.entrySet()) {
            String value = entry.getValue().getText().toString().trim();
            sendLine(entry.getKey() + "=" + value + "\n", true);
            sleepMs(30);
        }
    }

    private void sendMappingControllerValuesBlocking() {
        saveMappingSettings();
        sendLine("MapP=" + mapPInput.getText().toString().trim() + "\n", true);
        sleepMs(18);
        sendLine("MapI=" + mapIInput.getText().toString().trim() + "\n", true);
        sleepMs(18);
        sendLine("MapD=" + mapDInput.getText().toString().trim() + "\n", true);
        sleepMs(18);
        sendLine("MapSpeed=" + mapSpeedInput.getText().toString().trim() + "\n", true);
    }

    private void confirmBluetoothNameNow() {
        new AlertDialog.Builder(this)
                .setTitle("Try AT now")
                .setMessage("Use this only when the module is in AT mode. The BT link can drop after rename.")
                .setPositiveButton("Try", (dialog, which) -> sendBluetoothName(true))
                .setNegativeButton("Cancel", null)
                .show();
    }

    private void sendBluetoothName(boolean tryNow) {
        String name = btNameInput.getText().toString().trim();
        if (!isValidBluetoothName(name)) {
            toast("Use 1-20 chars: A-Z 0-9 space _ - .");
            return;
        }

        prefs().edit().putString(KEY_BT_NAME, name).apply();
        String command = tryNow ? "BtNameNow=" : "BtName=";
        if (btNameStatusText != null) {
            btNameStatusText.setText(tryNow ? "Trying AT rename..." : "Saved. Reboot robot in KEY/AT mode.");
        }
        new Thread(() -> sendLine(command + name + "\n", true)).start();
    }

    private boolean isValidBluetoothName(String name) {
        if (name == null || name.length() == 0 || name.length() > BT_NAME_MAX_LEN) {
            return false;
        }
        for (int i = 0; i < name.length(); i++) {
            char c = name.charAt(i);
            boolean allowed = (c >= 'A' && c <= 'Z') ||
                    (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') ||
                    c == ' ' || c == '_' || c == '-' || c == '.';
            if (!allowed) {
                return false;
            }
        }
        return true;
    }

    private void handleBluetoothNameStatus(String line) {
        if (btNameStatusText == null) {
            return;
        }

        if (line.startsWith("BT_NAME_PENDING,")) {
            btNameStatusText.setText("Pending. Reboot robot in KEY/AT mode.");
        } else if (line.startsWith("BT_NAME_OK,")) {
            btNameStatusText.setText("Renamed. Remove pairing and pair again.");
        } else if (line.startsWith("BT_NAME_ERROR,at_no_response")) {
            btNameStatusText.setText("No AT response. Enter KEY/AT mode and reboot.");
        } else {
            btNameStatusText.setText(line);
        }
    }

    private void sendCommand(String key, String value) {
        if (value == null || value.trim().isEmpty()) {
            toast("Missing value for " + key);
            return;
        }
        sendLine(key + "=" + value.trim() + "\n", true);
    }

    private void sendLine(String command, boolean showLog) {
        OutputStream classicStream = classicOutputStream;
        if (classicStream != null) {
            if (showLog) {
                appendLog("> " + command);
            }
            byte[] bytes = command.getBytes(StandardCharsets.US_ASCII);
            new Thread(() -> {
                try {
                    synchronized (writeLock) {
                        classicStream.write(bytes);
                        classicStream.flush();
                    }
                } catch (IOException ex) {
                    mainHandler.post(() -> {
                        setStatus("Disconnected");
                        appendLog("> Classic write failed: " + ex.getMessage() + "\n");
                    });
                }
            }, "gruzik-classic-write").start();
            return;
        }
        if (bluetoothGatt == null || commandCharacteristic == null) {
            appendLog("> not connected: " + command);
            return;
        }
        byte[] bytes = command.getBytes(StandardCharsets.US_ASCII);
        if (bytes.length > 180) {
            showError("BLE send failed", "Command exceeds negotiated packet size");
            return;
        }
        synchronized (writeLock) {
            pendingWrites.addLast(bytes);
        }
        if (showLog) {
            appendLog("> " + command);
        }
        mainHandler.post(this::writeNextBleCommand);
    }

    private void writeNextBleCommand() {
        BluetoothGatt gatt = bluetoothGatt;
        BluetoothGattCharacteristic characteristic = commandCharacteristic;
        byte[] bytes;
        synchronized (writeLock) {
            if (writeInProgress || pendingWrites.isEmpty() || gatt == null || characteristic == null) {
                return;
            }
            bytes = pendingWrites.peekFirst();
            writeInProgress = true;
        }

        boolean accepted;
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                accepted = gatt.writeCharacteristic(
                        characteristic,
                        bytes,
                        BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT) == BluetoothGatt.GATT_SUCCESS;
            } else {
                characteristic.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
                characteristic.setValue(bytes);
                accepted = gatt.writeCharacteristic(characteristic);
            }
        } catch (SecurityException ex) {
            accepted = false;
        }

        if (!accepted) {
            synchronized (writeLock) {
                if (!pendingWrites.isEmpty()) {
                    pendingWrites.removeFirst();
                }
                writeInProgress = false;
            }
            showError("BLE send failed", "Android rejected the GATT write");
            mainHandler.post(this::writeNextBleCommand);
        }
    }

    private boolean optimizeRecordedRoute() {
        return optimizeRecordedRoute(true);
    }

    private boolean optimizeRecordedRoute(boolean showStatus) {
        if (recordedRoute.size() < 2) {
            if (showStatus) {
                toast("Najpierw pobierz zapisana trase");
            }
            return false;
        }
        saveMappingSettings();

        int step = clampInt(readInt(optStepInput, 80), 1, 500);
        int smoothing = clampInt(readInt(optSmoothInput, 5), 1, 101);
        float baseSpeed = readFloat(optBaseSpeedInput, 40.0f);
        float gain = readFloat(optGainInput, 120.0f);
        float minSpeed = readFloat(optMinSpeedInput, 35.0f);
        float maxSpeed = readFloat(optMaxSpeedInput, 120.0f);
        if (maxSpeed < minSpeed) {
            float tmp = maxSpeed;
            maxSpeed = minSpeed;
            minSpeed = tmp;
        }

        boolean closedInput = isRecordedLoopClosed(recordedRoute);
        ArrayList<RoutePoint> source = loopSourceForOptimization(recordedRoute);
        if (source.size() < 2) {
            if (showStatus) {
                toast("Zapisana trasa jest za krotka");
            }
            return false;
        }

        ArrayList<RoutePoint> sampled = new ArrayList<>();
        for (int i = 0; i < source.size(); i += step) {
            sampled.add(source.get(i));
        }
        RoutePoint last = source.get(source.size() - 1);
        if (sampled.isEmpty() || distance(sampled.get(sampled.size() - 1), last) > 0.001f) {
            sampled.add(last);
        }

        ArrayList<RoutePoint> smoothed = closedInput ? smoothClosedRoute(sampled, smoothing) : smoothRoute(sampled, smoothing);
        optimizedRoute.clear();
        for (int i = 0; i < smoothed.size(); i++) {
            RoutePoint point = smoothed.get(i);
            RoutePoint previous = closedInput ? smoothed.get((i + smoothed.size() - 1) % smoothed.size()) : smoothed.get(Math.max(0, i - 1));
            RoutePoint next = closedInput ? smoothed.get((i + 1) % smoothed.size()) : smoothed.get(Math.min(smoothed.size() - 1, i + 1));
            float turn = turnAmount(previous, point, next);
            float speed = clampFloat(maxSpeed - (gain * turn), minSpeed, maxSpeed);
            if (turn < 0.20f) {
                speed = Math.max(speed, Math.min(baseSpeed, maxSpeed));
            }
            speed = clampFloat(speed, minSpeed, maxSpeed);
            optimizedRoute.add(new RoutePoint(point.x, point.y, speed));
        }
        updateRouteUi();
        if (showStatus) {
            transferText.setText("Optimized locally: " + optimizedRoute.size() + " pts / PID finish");
        }
        return true;
    }

    private ArrayList<RoutePoint> loopSourceForOptimization(List<RoutePoint> route) {
        ArrayList<RoutePoint> source = new ArrayList<>();
        if (route.isEmpty()) {
            return source;
        }

        source.addAll(route);
        if (source.size() > 2 && distance(source.get(0), source.get(source.size() - 1)) < 0.08f) {
            source.remove(source.size() - 1);
        }
        return source;
    }

    private boolean isRecordedLoopClosed(List<RoutePoint> route) {
        return route.size() > 2 && distance(route.get(0), route.get(route.size() - 1)) < 0.08f;
    }

    private float turnAmount(RoutePoint previous, RoutePoint point, RoutePoint next) {
        float a = (float) Math.atan2(point.y - previous.y, point.x - previous.x);
        float b = (float) Math.atan2(next.y - point.y, next.x - point.x);
        return Math.abs(normalizeAngle(b - a)) / (float) Math.PI;
    }

    private float normalizeAngle(float angle) {
        while (angle > Math.PI) {
            angle -= (float) (2.0 * Math.PI);
        }
        while (angle < -Math.PI) {
            angle += (float) (2.0 * Math.PI);
        }
        return angle;
    }

    private ArrayList<RoutePoint> smoothClosedRoute(List<RoutePoint> input, int window) {
        ArrayList<RoutePoint> output = new ArrayList<>();
        int countWindow = Math.min(Math.max(1, window), input.size());
        int radius = countWindow / 2;
        for (int i = 0; i < input.size(); i++) {
            float x = 0.0f;
            float y = 0.0f;
            for (int j = 0; j < countWindow; j++) {
                int index = (i - radius + j + input.size()) % input.size();
                x += input.get(index).x;
                y += input.get(index).y;
            }
            output.add(new RoutePoint(x / countWindow, y / countWindow, 0.0f));
        }
        return output;
    }

    private ArrayList<RoutePoint> smoothRoute(List<RoutePoint> input, int window) {
        ArrayList<RoutePoint> output = new ArrayList<>();
        int radius = Math.max(0, window / 2);
        for (int i = 0; i < input.size(); i++) {
            int from = Math.max(0, i - radius);
            int to = Math.min(input.size() - 1, i + radius);
            float x = 0.0f;
            float y = 0.0f;
            int count = 0;
            for (int j = from; j <= to; j++) {
                x += input.get(j).x;
                y += input.get(j).y;
                count++;
            }
            output.add(new RoutePoint(x / count, y / count, 0.0f));
        }
        return output;
    }

    private void uploadOptimizedRoute() {
        if (optimizedRoute.isEmpty() && !optimizeRecordedRoute(true)) {
            return;
        }

        new Thread(this::uploadOptimizedRouteBlocking).start();
    }

    private boolean uploadOptimizedRouteBlocking() {
        if (uploadRunning) {
            mainHandler.post(() -> toast("Upload already running"));
            return false;
        }
        if (bluetoothGatt == null || commandCharacteristic == null) {
            mainHandler.post(() -> toast("Not connected"));
            return false;
        }

        ArrayList<RoutePoint> routeToUpload = new ArrayList<>(optimizedRoute);
        if (routeToUpload.isEmpty()) {
            mainHandler.post(() -> toast("No optimized route"));
            return false;
        }

        uploadRunning = true;
        try {
            int count = routeToUpload.size();
            mainHandler.post(() -> transferText.setText("Uploading map.txt: 0/" + count));
            sendLine("MapUploadBegin=" + count + "\n", true);
            sleepMs(160);
            for (int i = 0; i < count; i++) {
                RoutePoint p = routeToUpload.get(i);
                String line = String.format(Locale.US, "MapPoint=%.3f,%.3f,%.3f\n", p.x, p.y, p.speed);
                sendLine(line, false);
                int progress = i + 1;
                if (progress % 25 == 0 || progress == count) {
                    mainHandler.post(() -> transferText.setText("Uploading map.txt: " + progress + "/" + count));
                }
                sleepMs(35);
            }
            sendLine("MapUploadEnd=1\n", true);
            mainHandler.post(() -> transferText.setText("Uploaded map.txt: " + count + " pts"));
            return true;
        } finally {
            uploadRunning = false;
        }
    }

    private void updateRouteUi() {
        if (routeMapView != null) {
            routeMapView.setRoutes(recordedRoute, optimizedRoute, robotOptimizedRoute);
        }
        if (routeInfoText != null) {
            routeInfoText.setText(String.format(Locale.US,
                    "Saved: %d pts / %.2f m   Optimized: %d pts / %.2f m   map.txt: %d pts",
                    recordedRoute.size(), routeLength(recordedRoute),
                    optimizedRoute.size(), routeLength(optimizedRoute),
                    robotOptimizedRoute.size()));
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

    private void setDefaultValues() {
        for (Map.Entry<String, EditText> entry : fields.entrySet()) {
            String key = entry.getKey();
            String value;
            if ("Kp".equals(key)) value = "0.015";
            else if ("Kd".equals(key)) value = "0.55";
            else if ("Base_speed".equals(key)) value = "125";
            else if ("Max_speed".equals(key)) value = "200";
            else if ("Sharp_bend_speed_right".equals(key)) value = "-75";
            else if ("Sharp_bend_speed_left".equals(key)) value = "120";
            else if ("Bend_speed_right".equals(key)) value = "-75";
            else if ("Bend_speed_left".equals(key)) value = "120";
            else if ("Treshold".equals(key)) value = "3300";
            else value = "0";
            entry.getValue().setText(value);
        }
        presetNameInput.setText("Default");
    }

    private void loadPresetNames() {
        presetNames.clear();
        String csv = prefs().getString(KEY_NAMES, "");
        if (csv != null && !csv.trim().isEmpty()) {
            String[] parts = csv.split("\\|", -1);
            for (String part : parts) {
                String name = part.trim();
                if (!name.isEmpty()) {
                    presetNames.add(name);
                }
            }
        }
        updatePresetSpinner();
    }

    private void saveCurrentPreset() {
        String name = presetNameInput.getText().toString().trim();
        if (name.isEmpty()) {
            toast("Preset name is empty");
            return;
        }

        SharedPreferences.Editor editor = prefs().edit();
        for (Map.Entry<String, EditText> entry : fields.entrySet()) {
            editor.putString(keyFor(name, entry.getKey()), entry.getValue().getText().toString().trim());
        }

        if (!presetNames.contains(name)) {
            presetNames.add(name);
        }
        editor.putString(KEY_NAMES, joinPresetNames());
        editor.apply();
        updatePresetSpinner();
        selectPreset(name);
        toast("Preset saved");
    }

    private void loadPreset(String name) {
        if (name == null || name.trim().isEmpty()) {
            return;
        }
        presetNameInput.setText(name);
        for (Map.Entry<String, EditText> entry : fields.entrySet()) {
            String current = entry.getValue().getText().toString();
            String value = prefs().getString(keyFor(name, entry.getKey()), current);
            entry.getValue().setText(value);
        }
    }

    private void deleteSelectedPreset() {
        String name = presetNameInput.getText().toString().trim();
        if (name.isEmpty() || !presetNames.contains(name)) {
            toast("No saved preset selected");
            return;
        }

        new AlertDialog.Builder(this)
                .setTitle("Delete preset")
                .setMessage(name)
                .setPositiveButton("Delete", (dialog, which) -> {
                    SharedPreferences.Editor editor = prefs().edit();
                    for (String key : fields.keySet()) {
                        editor.remove(keyFor(name, key));
                    }
                    presetNames.remove(name);
                    editor.putString(KEY_NAMES, joinPresetNames());
                    editor.apply();
                    updatePresetSpinner();
                    if (!presetNames.isEmpty()) {
                        loadPreset(presetNames.get(0));
                    } else {
                        setDefaultValues();
                    }
                })
                .setNegativeButton("Cancel", null)
                .show();
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

    private String keyFor(String presetName, String fieldName) {
        return "preset." + presetName + "." + fieldName;
    }

    private void loadMappingSettings() {
        SharedPreferences store = prefs();
        setText(cleanSpeedInput, store.getString(KEY_CLEAN_SPEED, "170"));
        setText(optStepInput, store.getString(KEY_OPT_STEP, "80"));
        setText(optSmoothInput, store.getString(KEY_OPT_SMOOTH, "5"));
        setText(optBaseSpeedInput, store.getString(KEY_OPT_BASE_SPEED, "40"));
        setText(optGainInput, store.getString(KEY_OPT_GAIN, "60"));
        setText(optMinSpeedInput, store.getString(KEY_OPT_MIN_SPEED, "35"));
        setText(optMaxSpeedInput, store.getString(KEY_OPT_MAX_SPEED, "95"));
        setText(mapPInput, store.getString(KEY_MAP_P, "90"));
        setText(mapIInput, store.getString(KEY_MAP_I, "0"));
        setText(mapDInput, store.getString(KEY_MAP_D, "10"));
        setText(mapSpeedInput, store.getString(KEY_MAP_SPEED, "80"));
        setText(joystickSpeedInput, store.getString(KEY_JOYSTICK_SPEED, "85"));
        setText(btNameInput, store.getString(KEY_BT_NAME, "GRUZIK4.0"));
    }

    private void saveMappingSettings() {
        if (cleanSpeedInput == null || optStepInput == null || mapPInput == null) {
            return;
        }
        prefs().edit()
                .putString(KEY_CLEAN_SPEED, cleanSpeedInput.getText().toString().trim())
                .putString(KEY_OPT_STEP, optStepInput.getText().toString().trim())
                .putString(KEY_OPT_SMOOTH, optSmoothInput.getText().toString().trim())
                .putString(KEY_OPT_BASE_SPEED, optBaseSpeedInput.getText().toString().trim())
                .putString(KEY_OPT_GAIN, optGainInput.getText().toString().trim())
                .putString(KEY_OPT_MIN_SPEED, optMinSpeedInput.getText().toString().trim())
                .putString(KEY_OPT_MAX_SPEED, optMaxSpeedInput.getText().toString().trim())
                .putString(KEY_MAP_P, mapPInput.getText().toString().trim())
                .putString(KEY_MAP_I, mapIInput.getText().toString().trim())
                .putString(KEY_MAP_D, mapDInput.getText().toString().trim())
                .putString(KEY_MAP_SPEED, mapSpeedInput.getText().toString().trim())
                .putString(KEY_JOYSTICK_SPEED, joystickSpeedInput.getText().toString().trim())
                .putString(KEY_BT_NAME, btNameInput == null ? "GRUZIK4.0" : btNameInput.getText().toString().trim())
                .apply();
    }

    private void setText(EditText input, String value) {
        if (input != null && value != null) {
            input.setText(value);
        }
    }

    private void updatePresetSpinner() {
        if (presetSpinner == null) {
            return;
        }
        ArrayList<String> names = new ArrayList<>(presetNames);
        if (names.isEmpty()) {
            names.add("No saved presets");
        }
        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, names);
        presetSpinner.setAdapter(adapter);
    }

    private void selectPreset(String name) {
        int index = presetNames.indexOf(name);
        if (index >= 0) {
            presetSpinner.setSelection(index);
        }
    }

    private SharedPreferences prefs() {
        return getSharedPreferences(PREFS, Context.MODE_PRIVATE);
    }

    private void setStatus(String status) {
        statusText.setText(status);
    }

    private void appendLog(String value) {
        if (Looper.myLooper() != Looper.getMainLooper()) {
            mainHandler.post(() -> appendLog(value));
            return;
        }

        String current = logText.getText().toString();
        String next = current + value;
        if (next.length() > 6000) {
            next = next.substring(next.length() - 6000);
        }
        logText.setText(next);
    }

    private void updateVoltageFromRobotText(String value) {
        int index = value.indexOf("Battery");
        if (index < 0 || voltageText == null) {
            return;
        }

        String tail = value.substring(index).trim();
        StringBuilder number = new StringBuilder();
        for (int i = 0; i < tail.length(); i++) {
            char c = tail.charAt(i);
            if ((c >= '0' && c <= '9') || c == '.' || c == ',') {
                number.append(c == ',' ? '.' : c);
            } else if (number.length() > 0) {
                break;
            }
        }
        if (number.length() > 0) {
            voltageText.setText(number + " V");
        }
    }

    private void showError(String title, String message) {
        new AlertDialog.Builder(this)
                .setTitle(title)
                .setMessage(message == null ? "Unknown error" : message)
                .setPositiveButton("OK", null)
                .show();
    }

    private void toast(String message) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show();
    }

    private TextView sectionTitle(String text) {
        TextView label = label(text, 16, TEXT);
        label.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        return label;
    }

    private TextView label(String text, int sp, int color) {
        TextView label = new TextView(this);
        label.setText(text);
        label.setTextSize(sp);
        label.setTextColor(color);
        label.setPadding(0, dp(4), 0, dp(4));
        return label;
    }

    private TextView telemetryText(String text) {
        TextView label = label(text, 14, TEXT);
        label.setTypeface(Typeface.MONOSPACE);
        label.setPadding(dp(10), dp(10), dp(10), dp(10));
        label.setMinLines(2);
        label.setBackground(panelDrawable(SURFACE_2, STROKE));
        return label;
    }

    private EditText input(String hint) {
        EditText input = new EditText(this);
        input.setHint(hint);
        input.setTextColor(TEXT);
        input.setHintTextColor(MUTED);
        input.setSingleLine(true);
        input.setPadding(dp(10), 0, dp(10), 0);
        input.setBackground(panelDrawable(SURFACE_2, STROKE));
        return input;
    }

    private EditText smallInput(String value) {
        EditText input = input(value);
        input.setText(value);
        input.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL | InputType.TYPE_NUMBER_FLAG_SIGNED);
        input.setGravity(Gravity.CENTER);
        return input;
    }

    private Button button(String text) {
        Button button = new Button(this);
        button.setText(text);
        button.setTextSize(13);
        button.setTextColor(TEXT);
        button.setAllCaps(false);
        button.setBackground(buttonDrawable(SURFACE_2, SURFACE_PRESSED));
        button.setPadding(dp(8), 0, dp(8), 0);
        button.setMinHeight(dp(44));
        button.setOnTouchListener((view, event) -> {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                view.setAlpha(0.72f);
            } else if (event.getAction() == MotionEvent.ACTION_UP || event.getAction() == MotionEvent.ACTION_CANCEL) {
                view.setAlpha(1.0f);
            }
            return false;
        });
        return button;
    }

    private Button primaryButton(String text, int color) {
        Button button = button(text);
        button.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        button.setBackground(buttonDrawable(color, darker(color)));
        return button;
    }

    private TextView blockLabel(String text) {
        TextView label = label(text, 15, TEXT);
        label.setGravity(Gravity.CENTER);
        label.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        label.setBackground(panelDrawable(SURFACE_2, STROKE));
        return label;
    }

    private TextView blockValue(String text) {
        TextView label = label(text, 16, TEXT);
        label.setGravity(Gravity.CENTER);
        label.setBackground(panelDrawable(SURFACE_2, STROKE));
        return label;
    }

    private LinearLayout readoutPanel(String title, TextView value) {
        LinearLayout panel = panel();
        panel.addView(sectionTitle(title), matchWrap());
        panel.addView(value, matchWrap());
        return panel;
    }

    private LinearLayout twoReadouts(String leftLabel, TextView leftValue, String rightLabel, TextView rightValue) {
        LinearLayout outer = row();
        outer.addView(readoutBox(leftLabel, leftValue), new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        outer.addView(readoutBox(rightLabel, rightValue), new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        return outer;
    }

    private LinearLayout readoutBox(String title, TextView value) {
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        box.setPadding(dp(3), dp(3), dp(3), dp(3));
        TextView label = label(title, 12, MUTED);
        box.addView(label, matchWrap());
        box.addView(value, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(58)));
        return box;
    }

    private LinearLayout twoInputs(String leftLabel, EditText leftInput, String rightLabel, EditText rightInput) {
        LinearLayout outer = row();
        outer.addView(labeledInput(leftLabel, leftInput), new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        outer.addView(labeledInput(rightLabel, rightInput), new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f));
        return outer;
    }

    private LinearLayout labeledInput(String title, EditText input) {
        LinearLayout box = new LinearLayout(this);
        box.setOrientation(LinearLayout.VERTICAL);
        box.setPadding(dp(3), dp(3), dp(3), dp(3));
        TextView label = label(title, 12, MUTED);
        box.addView(label, matchWrap());
        box.addView(input, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(48)));
        return box;
    }

    private LinearLayout row() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        return row;
    }

    private LinearLayout panel() {
        LinearLayout panel = new LinearLayout(this);
        panel.setOrientation(LinearLayout.VERTICAL);
        panel.setPadding(dp(10), dp(10), dp(10), dp(10));
        panel.setBackground(panelDrawable(SURFACE, STROKE));
        LinearLayout.LayoutParams params = matchWrap();
        params.setMargins(0, dp(5), 0, dp(5));
        panel.setLayoutParams(params);
        return panel;
    }

    private GradientDrawable panelDrawable(int color, int stroke) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(color);
        drawable.setCornerRadius(dp(8));
        drawable.setStroke(dp(1), stroke);
        return drawable;
    }

    private StateListDrawable buttonDrawable(int normal, int pressed) {
        StateListDrawable states = new StateListDrawable();
        states.addState(new int[]{android.R.attr.state_pressed}, panelDrawable(pressed, STROKE));
        states.addState(new int[]{-android.R.attr.state_enabled}, panelDrawable(Color.rgb(45, 48, 54), Color.rgb(55, 58, 64)));
        states.addState(new int[]{}, panelDrawable(normal, STROKE));
        return states;
    }

    private int darker(int color) {
        return Color.rgb(Math.round(Color.red(color) * 0.72f),
                Math.round(Color.green(color) * 0.72f),
                Math.round(Color.blue(color) * 0.72f));
    }

    private LinearLayout.LayoutParams matchWrap() {
        return new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
    }

    private LinearLayout.LayoutParams weightWrap() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f);
        params.setMargins(dp(3), dp(3), dp(3), dp(3));
        return params;
    }

    private void addSpacer(LinearLayout root, int dp) {
        View spacer = new View(this);
        root.addView(spacer, new LinearLayout.LayoutParams(1, dp(dp)));
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
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
            return Float.parseFloat(tokens[index].trim().replace(',', '.'));
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
}
