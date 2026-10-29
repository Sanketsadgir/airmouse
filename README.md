ESP32 Air Mouse

An experimental air mouse built with an ESP32, an
MPU6050-compatible motion sensor, and Bluetooth Low Energy (BLE).
The ESP32 sends mouse movement and optional button events to a Windows
computer.

Status: Experimental. Test the firmware on your own hardware
before relying on it. Sensor orientation, cursor gain, and click
sensitivity may need adjustment.

Features

BLE HID mouse emulation using the BleMouse library.

Relative cursor movement based on changes in the sensor's estimated
pointing orientation.

Attempts to move the Windows cursor to the bottom-right corner after
a BLE connection is established.

Gyroscope bias calibration during startup.

Optional physical left- and right-click buttons.

Optional acceleration-change flick detection for a left click.

Serial Monitor diagnostics for orientation, acceleration changes,
and BLE connection status.

Uses an MPU sensor at I²C address 0x68; the firmware logs the
WHO_AM_I register and continues if the library's
testConnection() check fails, provided the direct I²C checks pass.

Hardware

ESP32 DevKit / ESP32-WROOM-32

MPU6050-compatible IMU module

USB cable for programming and power

Optional momentary push buttons for left and right mouse clicks

Jumper wires

Wiring

MPU6050 module   ESP32

VCC              Use the voltage supported by your particular breakout board
GND              GND
SDA              GPIO 21
SCL              GPIO 22

The code uses I²C address 0x68. If your board uses another address,
update MPU_ADDR and the register-read helper consistently.

Optional mouse buttons

The default firmware configuration is:

Function        ESP32 pin Button connection

Left click        GPIO 25 Connect the button between GPIO 25 and GND
Right click       GPIO 26 Connect the button between GPIO 26 and GND

The pins use the ESP32's internal pull-up resistors. To disable a
button, set its corresponding pin constant to -1.

Software requirements

Arduino IDE.

ESP32 board support installed in Arduino IDE.

The MPU6050 library that provides MPU6050.h and the methods used
by this sketch (initialize(), getMotion6(), getRotation(),
getAcceleration(), etc.).

The ESP32-BLE-Mouse library that provides BleMouse.h.

The exact library versions can affect compilation. If Arduino reports
that MPU6050.h or BleMouse.h is missing, install the matching
library and restart the IDE.

Upload and connect

Open ESP32_Air_Mouse.ino in Arduino IDE. If your local sketch has
a different filename, rename the .ino file to match its sketch
folder.

Select the correct ESP32 board and serial port.

Upload the sketch.

Open Tools → Serial Monitor and set the baud rate to 115200.

Keep the sensor still while the startup gyroscope calibration runs.

On Windows, open Settings → Bluetooth & devices → Add device →
Bluetooth.

Select ESP32 Air Mouse.

After it connects, the firmware sends relative movement reports
intended to push the cursor to the bottom-right corner. Move the
device to control the cursor.

The cursor homing routine uses repeated relative mouse reports; it does
not read the Windows screen resolution or obtain the cursor's actual
position. The SCREEN_WIDTH_PX and SCREEN_HEIGHT_PX constants remain
in the source but are not used to query Windows.

Configuration

The following constants near the top of the sketch are the main tuning
controls:

Constant                            Purpose

CURSOR_GAIN_X                     Horizontal cursor gain per degree
of orientation change

CURSOR_GAIN_Y                     Vertical cursor gain per degree of
orientation change

INVERT_CURSOR_X                   Reverses horizontal cursor
direction when true

INVERT_CURSOR_Y                   Reverses vertical cursor direction
when true

ENABLE_FLICK_CLICK                Enables acceleration-change flick
detection

FLICK_THRESHOLD_G                 Threshold used by flick-click
detection

CLICK_COOLDOWN_MS                 Minimum interval between detected
clicks

BTN_LEFT_PIN                      GPIO for the optional left-click
button; -1 disables it

BTN_RIGHT_PIN                     GPIO for the optional right-click
button; -1 disables it

If horizontal or vertical movement is reversed, change the corresponding
INVERT_CURSOR_X or INVERT_CURSOR_Y value. Increase cursor gain for
faster movement and decrease it for slower movement.

Troubleshooting

The ESP32 does not appear in Windows Bluetooth

Confirm the sketch uploaded successfully and the board is powered.

Open Serial Monitor at 115200 baud and look for BLE mouse started.

Check that the device is not already connected to another host.
Remove any stale pairing in Windows and try pairing again.

The cursor does not move

Confirm Serial Monitor reports BLE:Connected.

Move the sensor deliberately and check whether the reported
yaw/pitch values change.

Check SDA on GPIO 21, SCL on GPIO 22, GND, power, and the I²C
address.

Adjust CURSOR_GAIN_X and CURSOR_GAIN_Y if movement is present
but too small.

The cursor moves in the wrong direction

Toggle INVERT_CURSOR_X or INVERT_CURSOR_Y as appropriate, then
upload again.

Cursor drift or jitter

Keep the device still during startup gyroscope calibration.

Check that the sensor is firmly mounted and wires are secure.

Tune the filter and gain values cautiously. The orientation estimate
and cursor response depend on sensor quality and how the device is
held.

Compilation errors

Verify the required libraries are installed.

Make sure the .ino filename matches the sketch folder name.

Read the first compiler error; later errors may be consequences of
the first missing header or incompatible library API.

Limitations and safety

This is a hobby/experimental project, not a certified pointing or
accessibility device.

The firmware cannot verify the actual Windows cursor coordinates or
display resolution.

Flick-click detection may trigger accidentally depending on how the
sensor is moved. Disable it with ENABLE_FLICK_CLICK = false if
needed.

The source includes orientation processing that uses multiple sensor
axes. It should not be described as strictly X/Y-only processing
without changing and testing the firmware.

Do not use this project as a substitute for safety-critical
controls.

License

No license is specified by the provided source code. Add a license file
and update this section if you want others to reuse, modify, or
redistribute the project.
