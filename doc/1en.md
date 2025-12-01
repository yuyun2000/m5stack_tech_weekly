# WiFi Channels, Deep Sleep Glitches & NB-IoT Workarounds | M5Stack Dev Diary

Hey Makers! 👋

We've been digging through some interesting edge cases that came up in the community recently—ranging from HID keyboard quirks on Tab5 to button debouncing on Cardputer, and even a gnarly UART issue with the NB-IoT2 Unit. If you've been scratching your head over any of these, here's what we found. Let's dive in! 🛠️

---

## 01 | Tab5: Parsing Special Keys in BLE HID Keyboards

**Product**: Tab5 (C145/K145)  
**Category**: Bluetooth / HID Protocol

**The Issue:**  
You've got a BLE keyboard connected to your Tab5, and basic alphanumeric keys work fine—but arrow keys and function keys (F1-F12)? Nothing. Radio silence.

**Why This Happens:**  
Standard HID keyboard reports use distinct key code ranges for special keys. Function keys live in `0x3A-0x45`, and arrow keys hang out in `0x4F-0x52`. If your parsing logic only handles the basic alphanumeric range, these guys get ignored.

**The Fix:**  
Add the special key codes and extend your HID report parser:

```cpp
// Function keys (F1-F12)
#define HID_KEY_F1  0x3A
#define HID_KEY_F12 0x45

// Arrow keys
#define HID_KEY_ARROW_UP    0x52
#define HID_KEY_ARROW_DOWN  0x51
#define HID_KEY_ARROW_LEFT  0x50
#define HID_KEY_ARROW_RIGHT 0x4F
```

Then, in your HID input handler:

```cpp
void onInputReport(uint8_t reportId, uint16_t connHandle, uint8_t* data, uint16_t len) {
  if (len < 8) return;
  
  uint8_t keyCodes[6] = {data[2], data[3], data[4], data[5], data[6], data[7]};
  
  for (int i = 0; i < 6; i++) {
    if (keyCodes[i] == 0) continue;
    
    // Handle function keys (F1-F12)
    if (keyCodes[i] >= HID_KEY_F1 && keyCodes[i] <= HID_KEY_F12) {
      int fKeyNum = keyCodes[i] - HID_KEY_F1 + 1;
      M5.Display.printf("[F%d]", fKeyNum);
    }
    // Handle arrow keys
    else {
      switch(keyCodes[i]) {
        case HID_KEY_ARROW_UP:    M5.Display.print("[UP]"); break;
        case HID_KEY_ARROW_DOWN:  M5.Display.print("[DOWN]"); break;
        case HID_KEY_ARROW_LEFT:  M5.Display.print("[LEFT]"); break;
        case HID_KEY_ARROW_RIGHT: M5.Display.print("[RIGHT]"); break;
      }
    }
  }
}
```

**Pro Tips:**  
- HID key codes follow the USB HID Usage Tables spec—keyboard vendors stick to this religiously.
- Arrow and function keys are *not* affected by modifier keys (Shift, Ctrl, etc.). No combo handling needed here.
- Want to add Home/End/PageUp? They're in `0x4A-0x4E`. Same approach.
- Media keys (volume, play/pause) live in a separate Consumer Control report (UUID `0x0C`). That's a whole different beast.

---

## 02 | Cardputer-Adv: Debouncing the Dual Button Unit

**Product**: Cardputer-Adv (K132-Adv) / Dual Button Unit (U025)  
**Category**: Sensor / Signal Processing

**The Issue:**  
You're reading button states directly from GPIO, and it's chaos. One press registers as three. One release registers as five. You're going insane.

**Why This Happens:**  
Mechanical buttons are analog demons. When you press or release, the contacts bounce for 5-50ms, creating electrical noise that looks like multiple rapid presses to your microcontroller. Polling GPIO directly = you get all that noise.

**The Fix:**  
Use M5Unified's `Button_Class`. It handles debouncing in software, so you get clean, stable events.

**Step 1:** Initialize your buttons with debounce thresholds.

```cpp
#include <M5Unified.h>

#define BLUE_BTN_PIN 1  // G1 = Blue button
#define RED_BTN_PIN  2  // G2 = Red button

m5::Button_Class blueBtn;
m5::Button_Class redBtn;

void setup() {
  M5Cardputer.begin();
  
  // Set debounce time (20ms is a sweet spot for most buttons)
  blueBtn.setDebounceThresh(20);
  redBtn.setDebounceThresh(20);
  
  // Bind to GPIO (false = button is LOW when pressed)
  blueBtn.begin(BLUE_BTN_PIN, false);
  redBtn.begin(RED_BTN_PIN, false);
}
```

**Step 2:** Check for stable events in your main loop.

```cpp
void loop() {
  M5Cardputer.update();
  blueBtn.update();  // Internal debounce sampling
  redBtn.update();
  
  // Detect press (fires once)
  if (blueBtn.wasPressed()) {
    Serial.println("Blue Button Pressed");
  }
  
  // Detect release
  if (redBtn.wasReleased()) {
    Serial.println("Red Button Released");
  }
  
  // Detect click (full press + release)
  if (blueBtn.wasClicked()) {
    Serial.println("Blue Button Clicked");
  }
  
  delay(5);
}
```

**Available Debounced Events:**  
- `wasPressed()` — Fires once on press
- `wasReleased()` — Fires once on release
- `wasClicked()` — Fires once after a full press-and-release
- `isPressed()` — True while held down
- `wasHold()` — Fires after a long press (threshold configurable with `setHoldThresh(ms)`)

**Pro Tips:**  
- Keep debounce thresholds between 20-50ms. Too low = noise slips through. Too high = sluggish response.
- The Dual Button Unit already has 10kΩ hardware pull-ups. Don't use `INPUT_PULLUP`.
- `Button_Class` uses continuous sampling to confirm state stability—way better than a simple `delay()`.
- Default long-press threshold is 1000ms. Adjust with `setHoldThresh()` if needed.

---

## 03 | StamPLC: Direct UART Control for NB-IoT2 Unit (SIM7028)

**Product**: StamPLC (K141) / NB-IoT2 Unit (U118)  
**Category**: Compilation Error / Library Compatibility

**The Issue:**  
You're using UIFlow 2.3.8 firmware on StamPLC, and the `NBIOT2Unit` class throws an `AttributeError` when you try to call `execute_at_command()` or `execute_at_command2()`. You can't even send a basic AT command to configure the APN.

**Why This Happens:**  
Early StamPLC firmware (<2.4.0) shipped with a broken `NBIOT2Unit` driver. The method signatures were misaligned—the library expected some internal data structure instead of strings or tuples. This is fixed in later firmware, but there's no official update path yet for 2.3.8 users.

**The Workaround:**  
Bypass the buggy library entirely. Talk to the SIM7028 module directly over UART2 using Arduino's `HardwareSerial`.

**Step 1:** Initialize UART2 with StamPLC's Port C pins.

```cpp
#include <M5StamPLC.h>
#include <HardwareSerial.h>

// Port C mapping: RX=GPIO16, TX=GPIO17
HardwareSerial sim7028Serial(2);

void setup() {
  M5StamPLC.begin();
  Serial.begin(115200);  // Debug output
  
  sim7028Serial.begin(115200, SERIAL_8N1, 16, 17);  // Init UART2
  delay(2000);
}
```

**Step 2:** Create an AT command helper.

```cpp
String sendATCommand(String cmd, int timeout = 1000) {
  sim7028Serial.print(cmd + "\r\n");  // SIM7028 requires \r\n
  
  long start = millis();
  String response = "";
  
  while (millis() - start < timeout) {
    if (sim7028Serial.available()) {
      response += sim7028Serial.readString();
    }
  }
  
  return response;
}
```

**Step 3:** Test basic AT commands.

```cpp
void loop() {
  // Basic comms test
  Serial.println(sendATCommand("AT"));  // Should return "OK"
  
  // Signal strength
  Serial.println(sendATCommand("AT+CSQ"));  // Returns +CSQ: <rssi>,<ber>
  
  // Network registration status
  Serial.println(sendATCommand("AT+CEREG?"));  // +CEREG: 0,1 = registered
  
  delay(5000);
}
```

**Step 4:** Configure APN and activate network (full flow).

```cpp
// Set APN (replace with your carrier's APN)
sendATCommand("AT+CGDCONT=1,\"IP\",\"telstra.m2m\"");
delay(1000);

// Activate PDP context
sendATCommand("AT+CGACT=1,1");
delay(2000);

// Attach to GPRS network
sendATCommand("AT+CGATT=1");
delay(5000);

// Verify IP address assignment
Serial.println(sendATCommand("AT+CGPADDR=1"));
```

**Pro Tips:**  
- StamPLC's Port C uses UART2 with fixed pins: RX=16, TX=17. No remapping allowed.
- SIM7028 *requires* `\r\n` line endings. Without them, commands are ignored.
- Signal strength (CSQ) ranges from 0-31. (99 = unknown). You want ≥10 before trying to attach.
- Network registration can take 5-30 seconds. `AT+CEREG?` returning `+CEREG: 0,1` means success.
- This approach also works in UIFlow 2's MicroPython environment—just swap `HardwareSerial` for `machine.UART`.
- **LED indicators:** Blue LED blinking = searching for network. Solid = connected. Off = module unpowered or dead.

---

## 💬 Discussion

Have you run into similar gotchas with HID parsing, button debouncing, or cellular modules? Drop a comment below or share your own workarounds! We'd love to hear how you're tackling these issues in the wild.

## 📌 Resources

- 📚 **Docs**: [https://docs.m5stack.com](https://docs.m5stack.com)  
- 🗣️ **Forum**: [https://community.m5stack.com](https://community.m5stack.com)  
- 🛒 **Shop**: [https://shop.m5stack.com](https://shop.m5stack.com)

---

*Note: This series shares common troubleshooting tips from the M5Stack engineering desk. All user data has been anonymized.*

