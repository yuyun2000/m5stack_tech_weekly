# WiFi Glitches, Linux Kernel Woes & Multi-Encoder Hacks | M5Stack Dev Diary

Hey Makers! 👋

We've been diving into some gnarly troubleshooting cases lately—from button callbacks on the Fire v2.7 to wrestling with Linux 6.1 kernel drivers, and even wiring up multiple encoders for Matter lighting. If you've been banging your head against similar issues, this one's for you. Let's break it down! 🛠️

---

## 01 | Fire v2.7 Button Events in UiFlow2 Python

**Product:** M5Stack Fire v2.7 (K007-V27)  
**Category:** Firmware / Library Compatibility

**The Problem:**  
You're trying to use button callbacks on the Fire v2.7 with UiFlow2 Python, but you keep getting `ImportError: no module named 'm5ui'` or `no module named 'm5stack'`. Sound familiar?

**What's Going On:**  
This happens when you've flashed a generic MicroPython firmware instead of the UiFlow2-specific one. The generic build doesn't include the `m5ui` or `M5Unified` modules. Also, some folks are using outdated import syntax (like `from m5stack import M5`), which is no longer supported.

**The Fix:**  
Use the **M5Unified** library correctly. Here's a clean example with short/long press detection:

```python
import M5
from M5 import *
import time

# Initialize M5 hardware (MUST be first!)
M5.begin()

# Define button callback (supports short/long press)
def btnC_callback(state):
    start_time = time.ticks_ms()
    
    # Measure press duration (max 4s to prevent lockup)
    while M5.BtnC.isPressed() and time.ticks_diff(time.ticks_ms(), start_time) < 4000:
        time.sleep_ms(50)
    
    press_time = time.ticks_diff(time.ticks_ms(), start_time)
    
    # Short press (150ms-1200ms)
    if 150 <= press_time < 1200:
        print(f"Short press ({press_time}ms)")
        # Add your action here (e.g., send sensor data)
    
    # Long press (≥1800ms)
    elif press_time >= 1800:
        print(f"Long press ({press_time}ms)")
        # Add your action here (e.g., trigger camera)

# Register the callback (WAS_PRESSED fires on button down)
M5.BtnC.setCallback(type=BtnC.CB_TYPE.WAS_PRESSED, cb=btnC_callback)

# Main loop (M5.update() is REQUIRED for event handling)
while True:
    M5.update()  # Process button events and hardware state
    time.sleep_ms(50)
```

**Real-World Example: Button + WiFi + Camera:**

```python
import M5
from M5 import *
import time
import wifi  # Your custom WiFi module
import cam   # Your custom camera module
import svr   # Your custom server module

M5.begin()

def btnC_callback(state):
    start = time.ticks_ms()
    while M5.BtnC.isPressed() and time.ticks_diff(time.ticks_ms(), start) < 4000:
        time.sleep_ms(50)
    press = time.ticks_diff(time.ticks_ms(), start)
    
    # Short press: Send sensor data
    if 150 <= press < 1200:
        print("[Short] Sending sensor data")
        # send.add_to_queue()
        # send.send_queue()
        return
    
    # Long press: Capture photos and upload
    if press >= 1800:
        print("[Long] Starting photo capture")
        
        # Switch to camera WiFi
        if not wifi.to_cam():
            print("WiFi switch failed")
            wifi.to_svr()  # Revert to server network
            return
        
        time.sleep(1)  # Let WiFi stabilize
        
        # Capture from multiple cameras
        images_ok = []
        for cam_no in [1, 2, 3]:
            if cam.snap(cam_no):
                images_ok.append(cam_no)
                print(f"CAM{cam_no} captured")
            time.sleep(0.3)
        
        # Switch back to server WiFi
        wifi.to_svr()
        time.sleep(1)
        
        # Upload images
        for cam_no in images_ok:
            svr.up(cam_idx=cam_no)
            print(f"CAM{cam_no} uploaded")

M5.BtnC.setCallback(type=BtnC.CB_TYPE.WAS_PRESSED, cb=btnC_callback)

while True:
    M5.update()
    time.sleep_ms(50)
```

**Gotchas:**
- **Flash the right firmware!** Use M5Burner to install the **UiFlow2 Fire-specific firmware** (v1.1.0+). Generic MicroPython won't work.
- `M5.begin()` MUST be called before any M5 operations, or your button objects won't initialize.
- Avoid blocking operations (like long `time.sleep()` calls) inside callbacks—they'll freeze the main loop.
- When switching WiFi networks, add 1-2 second delays to ensure stable connections. Check connection status after `wifi.to_cam()`.
- The 50ms sleep in the main loop balances CPU usage and button responsiveness. Don't go below 10ms.

---

## 02 | LLM-8850 Card Driver Fails on Linux 6.1 Kernel

**Product:** LLM-8850 Card (K151)  
**Category:** Kernel Compatibility / Driver Compilation

**The Problem:**  
You're trying to install the `axclhost` driver on a Linux 6.1.75 kernel via `apt`, but DKMS compilation fails with:  
`ERROR: kernel package linux-headers-6.1.75 is not supported`

**What's Going On:**  
The precompiled `axclhost` driver package (v3.6.5) uses DKMS for dynamic compilation, but its source code only supports Linux 5.15.x and 6.8.x kernels. Linux 6.1.75 has incompatible PCIe interrupt management APIs (like `pci_alloc_irq_vectors`), causing the build to fail.

**The Fix:**

**Step 1: Nuke the broken installation**

```bash
# Force remove the corrupted package
sudo dpkg --force-remove-reinstreq --purge axclhost

# Clean up DKMS cache and driver source
sudo rm -rf /var/lib/dkms/axclhost/
sudo rm -rf /usr/src/axclhost-1.0/

# Remove unused kernel packages
sudo apt autoremove -y && sudo apt clean
```

**Step 2: Verify kernel headers are installed**

```bash
# Install headers and build tools for your kernel
sudo apt install -y linux-headers-$(uname -r) build-essential dkms

# Verify headers path
ls /lib/modules/$(uname -r)/build  # Should show Makefile, etc.
```

**Step 3: Manually compile the driver from AX SDK (Recommended)**

```bash
# Download and extract AX650 SDK (replace with actual version)
wget https://repo.llm.m5stack.com/ax_sdk/AX650_SDK_V2.23.1.tgz
tar -xvf AX650_SDK_V2.23.1.tgz
cd AX650_SDK_V2.23.1

# Unpack SDK and pull kernel sources
./sdk_unpack.sh

# Compile host driver (ARM64 architecture)
cd axcl/build
make clean
make host=arm64 KERNEL_DIR=/lib/modules/$(uname -r)/build all -j$(nproc)

# Install driver module
sudo cp ../out/axcl_linux_arm64/driver/axcl_host.ko \
     /lib/modules/$(uname -r)/kernel/drivers/pci/
sudo depmod -a
sudo modprobe axcl_host
```

**Step 4: Verify driver loading**

```bash
# Check if driver is loaded
lsmod | grep axcl_host

# Use axcl-smi to detect device
axcl-smi

# Expected output:
# | Card  Name           Firmware | Bus-Id       | Memory-Usage          |
# |    0  AX650N           V3.6.5 | 0001:01:00.0 | 148 MiB / 945 MiB     |
```

**Step 5 (Advanced): Patch driver source for 6.1 kernel**

If compilation still fails, you'll need to adapt the driver source:

```bash
# Enter driver source directory
cd AX650_SDK_V2.23.1/drv/pcie/driver

# Edit axcl_host.c (example)
nano axcl_host.c

# Replace pci_enable_msi() with pci_alloc_irq_vectors()
# Reference: https://www.kernel.org/doc/html/v6.1/

# Rebuild
make clean && make host=arm64 KERNEL_DIR=/lib/modules/$(uname -r)/build all
```

**Gotchas:**
- Official precompiled drivers only support Linux 5.15.x and 6.8.x. For 6.1.x, manual compilation is required.
- If the SDK doesn't explicitly support 6.1, you'll need to patch the PCIe interrupt APIs in the driver source (e.g., replace `pci_enable_msi` with `pci_alloc_irq_vectors`).
- Check detailed logs on failure: `cat /var/lib/dkms/axclhost/1.0/build/make.log`
- For best compatibility, use the officially recommended Linux 5.15.73 kernel.
- Download links for AX SDK: Contact official support at support@m5stack.com.
- Driver compilation requires gcc-12 or higher: `sudo apt install gcc-12 g++-12`
- If you have multiple kernel versions, ensure you specify the correct `KERNEL_DIR` path during compilation.
- Before loading the driver, confirm PCIe device is detected: `lspci | grep AX650` should show device info.
- After loading, set device permissions: `sudo chmod 666 /dev/axcl*`

---

## 03 | Connecting Module Audio (M144) to Basic v2.7

**Product:** M5Stack Basic v2.7 (K001) / Module Audio (M144)  
**Category:** Hardware Wiring / Audio Driver Configuration

**The Problem:**  
You need to connect the Module Audio M144 to a Basic v2.7 for microphone input and speaker output, but you're not sure how to wire it or configure the drivers.

**What's Going On:**  
The M144 uses the M5-Bus interface to communicate with the Basic v2.7 via I2C (for controlling the STM32G030 and ES8388 codec) and I2S (for audio data transfer). You need to configure the I2S pin mapping (Mode A) and ES8388 registers correctly to get audio working.

**The Fix:**

**Hardware Connection:**

```
Direct Stack Connection:
- Stack the M144 module directly onto the bottom of the Basic v2.7 via M5-Bus
- No extra wiring needed (power and signals auto-connect through M5-Bus)
- Ensure M144's A/B mode jumper is set to Mode A (default for Basic v2.7)

Pin Mapping:
Basic v2.7 GPIO → M144 Function
G21 (SDA)        → I2C Data (controls STM32G030 and ES8388)
G22 (SCL)        → I2C Clock
G12              → I2S_BCK (audio clock)
G13              → I2S_LRCK (left/right channel select)
G15              → I2S_DOUT (speaker output)
G34              → I2S_DIN (microphone input)
```

**Arduino Code Example:**

```cpp
// Install required libraries via Arduino Library Manager:
// 1. M5Unified (Basic v2.7 hardware abstraction)
// 2. M5Module-Audio (M144 driver)

#include <M5Unified.h>
#include <M5ModuleAudio.h>
#include <driver/i2s.h>

M5ModuleAudio audio_module;
ES8388 es8388;

#define SAMPLE_RATE 44100
#define BUFFER_SIZE 1024

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  
  // Initialize M144 (I2C address 0x33)
  audio_module.begin(&Wire, 21, 22);  // SDA=G21, SCL=G22
  audio_module.setHPMode(AUDIO_HPMODE_CTIA);  // Set headphone standard
  audio_module.setMICStatus(AUDIO_MIC_OPEN);  // Enable microphone
  
  // Initialize ES8388 codec
  es8388.init(&Wire, 21, 22);
  es8388.setADCInput(ADC_INPUT_LINPUT1_RINPUT1);  // Mic input
  es8388.setDACOutput(DAC_OUTPUT_OUT1);           // Headphone output
  es8388.setSampleRate(SAMPLE_RATE_44K);
  
  // Configure I2S audio transfer
  i2s_config_t i2s_cfg = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
  };
  i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
  
  // Set I2S pins (Basic v2.7)
  i2s_pin_config_t pin_cfg = {
    .bck_io_num = 12,    // I2S_BCK
    .ws_io_num = 13,     // I2S_LRCK
    .data_out_num = 15,  // I2S_DOUT (speaker)
    .data_in_num = 34,   // I2S_DIN (microphone)
  };
  i2s_set_pin(I2S_NUM_0, &pin_cfg);
}

void loop() {
  M5.update();
  
  // Real-time audio passthrough (mic → speaker)
  uint16_t audio_buf[BUFFER_SIZE];
  size_t bytes_read;
  
  // Read from microphone
  i2s_read(I2S_NUM_0, audio_buf, sizeof(audio_buf), &bytes_read, portMAX_DELAY);
  
  // Write to speaker (add audio processing here if needed)
  i2s_write(I2S_NUM_0, audio_buf, sizeof(audio_buf), &bytes_read, portMAX_DELAY);
}
```

**Key Configuration Parameters:**

```cpp
// Headphone standard switch (choose based on your headphones)
audio_module.setHPMode(AUDIO_HPMODE_CTIA);  // CTIA standard (international)
// or
audio_module.setHPMode(AUDIO_HPMODE_OMTP);  // OMTP standard (some Chinese brands)

// Microphone gain adjustment (0dB~48dB)
es8388.setMicGain(MIC_GAIN_24DB);  // Start with 24dB

// RGB LED indicator control
audio_module.setRGBLED(0, 0x00FF00);  // Set to green (recording)
```

**Gotchas:**
- M144's A/B mode jumper MUST be set to Mode A (default for Basic v2.7). Mode B is for CoreS3 to avoid conflicts with onboard ES7210.
- I2C addresses: STM32G030 is 0x33, ES8388 is 0x10. Make sure there are no address conflicts on the bus.
- Recommended sample rate is 44.1kHz (CD quality). Lowering to 16kHz reduces CPU usage.
- Microphone input supports TRRS headphones (with mic) and TRS standalone mics. Switch via `setMICStatus()`.
- Check headphone insertion with `audio_module.getHPInsertStatus()` (returns 0=not inserted, 1=inserted).
- Buffer size (BUFFER_SIZE) affects latency and stability. Recommended range: 512-2048 bytes.
- If you hear noise, add a 0.1µF decoupling capacitor near the M144's power pins.
- I2S data transfer uses DMA. Avoid long blocking operations in `loop()`.
- ES8388 supports hardware EQ. Configure bass/treble boost via registers.
- To record to SD card, configure the file system separately (use M5.SD library).
- Power optimization: Turn off the mic when not in use via `audio_module.setMICStatus(AUDIO_MIC_CLOSE)`.

---

## 04 | Using PaHub v2.1 for Multi-Encoder Matter Lighting

**Product:** Unit Encoder (U135) / Nano C6 (C125) / Unit PaHub v2.1 (U040-B-V21)  
**Category:** Hardware Selection / I2C Address Conflict Resolution

**The Problem:**  
You want to connect 3 Unit Encoders to a Nano C6 for Matter over Thread smart lighting control, but all encoders share the same I2C address (0x40). How do you tell them apart?

**What's Going On:**  
Unit Encoder uses a fixed I2C address of 0x40. Connecting multiple encoders directly to the same I2C bus causes address conflicts, and the controller can't distinguish between devices. You need an I2C multiplexer or address reconfiguration to solve this.

**The Fix:**

**Use Unit PaHub v2.1 for I2C Channel Isolation:**

```
Hardware Connection:
Nano C6 Grove Port → Unit PaHub v2.1 Main Port
PaHub v2.1 CH0~CH2 → 3x Unit Encoders

How It Works:
- PaHub v2.1 uses a PCA9548AP chip to split 1 I2C port into 6 independent channels
- Each channel can connect devices with the same I2C address (0x40)
- Software switches channels to select devices (no need to modify encoder addresses)
```

**Arduino Code Example (Channel Switching + Encoder Reading):**

```cpp
#include <M5Unified.h>
#include <Wire.h>

// PaHub v2.1 default I2C address (adjustable via DIP switch: 0x70~0x77)
#define PAHUB_ADDR 0x70

// Unit Encoder default I2C address
#define ENCODER_ADDR 0x40

// Encoder register addresses
#define REG_ENCODER_VALUE 0x10  // Encoder count (int32)
#define REG_BUTTON_STATUS 0x20  // Button status (0=not pressed, 1=pressed)

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Wire.begin(1, 2);  // Nano C6 I2C pins (SDA=G1, SCL=G2)
}

// Switch PaHub channel (0~5)
void selectChannel(uint8_t channel) {
  Wire.beginTransmission(PAHUB_ADDR);
  Wire.write(1 << channel);  // Channel select bitmask
  Wire.endTransmission();
  delay(10);  // Wait for channel to stabilize
}

// Read encoder count value
int32_t readEncoderValue(uint8_t channel) {
  selectChannel(channel);
  
  Wire.beginTransmission(ENCODER_ADDR);
  Wire.write(REG_ENCODER_VALUE);
  Wire.endTransmission();
  
  Wire.requestFrom(ENCODER_ADDR, 4);
  int32_t value = 0;
  for (int i = 0; i < 4; i++) {
    value |= ((int32_t)Wire.read()) << (i * 8);
  }
  return value;
}

// Read button status
bool readButtonStatus(uint8_t channel) {
  selectChannel(channel);
  
  Wire.beginTransmission(ENCODER_ADDR);
  Wire.write(REG_BUTTON_STATUS);
  Wire.endTransmission();
  
  Wire.requestFrom(ENCODER_ADDR, 1);
  return Wire.read() == 1;
}

void loop() {
  // Read status of 3 encoders
  for (uint8_t ch = 0; ch < 3; ch++) {
    int32_t value = readEncoderValue(ch);
    bool pressed = readButtonStatus(ch);
    
    Serial.printf("Encoder %d: Value=%d, Button=%s\n", 
                  ch, value, pressed ? "Pressed" : "Released");
    
    // Send Matter commands based on encoder state (example)
    if (pressed) {
      // Button press: Toggle light on/off
      sendMatterCommand(ch, "OnOff", "Toggle");
    } else if (value != 0) {
      // Encoder rotation: Adjust brightness
      sendMatterCommand(ch, "LevelControl", value > 0 ? "Up" : "Down");
    }
  }
  
  delay(100);  // Polling interval
}

// Matter command function (implement based on your Matter library)
void sendMatterCommand(uint8_t lightID, const char* cluster, const char* action) {
  // Example: Send command to corresponding light via Matter over Thread
  Serial.printf("Send to Light %d: %s -> %s\n", lightID, cluster, action);
  // Actual implementation requires Matter SDK API calls
}
```

**UIFlow2 Implementation (Block Coding):**

```python
# Import PaHub and Encoder libraries
from unit import PaHubUnit, EncoderUnit
import time

# Initialize PaHub (I2C address 0x70)
pahub = PaHubUnit(i2c_addr=0x70)

# Initialize 3 encoders (via different PaHub channels)
encoder1 = EncoderUnit(pahub=pahub, channel=0)
encoder2 = EncoderUnit(pahub=pahub, channel=1)
encoder3 = EncoderUnit(pahub=pahub, channel=2)

while True:
    # Read encoder 1
    value1 = encoder1.get_rotary_value()
    button1 = encoder1.get_button_status()
    
    # Read encoder 2
    value2 = encoder2.get_rotary_value()
    button2 = encoder2.get_button_status()
    
    # Read encoder 3
    value3 = encoder3.get_rotary_value()
    button3 = encoder3.get_button_status()
    
    # Control Matter lights based on state (requires Matter library integration)
    if button1:
        matter.send_command(light_id=1, cluster="OnOff", action="Toggle")
    if value2 != 0:
        matter.send_command(light_id=2, cluster="LevelControl", value=value2)
    
    time.sleep(0.1)
```

**Gotchas:**
- PaHub v2.1's I2C address is adjustable via onboard DIP switch (0x70~0x77). You can cascade multiple PaHubs to expand up to 36 channels.
- Each channel switch requires 10ms stabilization time. Avoid frequent switching to prevent communication errors.
- PaHub v2.1 supports 6 channels max. 3 encoders occupy CH0~CH2. Remaining CH3~CH5 can be used for other I2C devices.
- Encoder power consumption is ~17mA each. 3 encoders total 51mA, which PaHub v2.1 can supply via Nano C6's Grove port (no external power needed).
- Matter over Thread command sending requires ESP Matter SDK integration (see [ESP Matter GitHub](https://github.com/espressif/esp-matter)).
- Encoder buttons support long-press detection. Poll the `REG_BUTTON_STATUS` register and use a timer to implement this.
- For faster response, use interrupt mode (connect encoder's INT pin to Nano C6's GPIO).
- PaHub v2.1 channel switching is done by writing a 1-byte bitmask (e.g., 0x01=CH0, 0x02=CH1, 0x04=CH2).
- Avoid selecting multiple channels simultaneously (non-single-bit bitmask)—it causes I2C bus conflicts.
- Encoder count value is a signed 32-bit integer. Clockwise rotation increments, counterclockwise decrements.
- Reset encoder count to 0 by writing to the `REG_ENCODER_VALUE` register.
- Matter device binding requires pairing within a Thread network (use Nano C6's Thread Border Router functionality).

---

## 💬 Discussion

Have you run into similar "gotchas" with your M5Stack projects? Found a clever workaround? Drop a comment below or share your own debugging stories!

## 📌 Resources

- 📚 **Docs**: [https://docs.m5stack.com](https://docs.m5stack.com)  
- 🗣️ **Forum**: [https://community.m5stack.com](https://community.m5stack.com)  
- 🛒 **Shop**: [https://shop.m5stack.com](https://shop.m5stack.com)

---

*Note: This series shares common troubleshooting tips from the M5Stack engineering desk. All user data has been anonymized.*

---

# Social Media Adaptation Strategy

## For Reddit (r/m5stack, r/esp32, r/embedded)

**Suggested Title:**  
*"Troubleshooting 4 Common M5Stack Issues: Fire v2.7 Buttons, Linux Kernel Drivers, Audio Modules & Multi-Encoder I2C"*

**Posting Advice:**  
- Post the full markdown directly (Reddit loves long-form technical content).  
- Add a **TL;DR** at the top:
  > **TL;DR:** Fixed button callbacks on Fire v2.7 with M5Unified, wrestled with Linux 6.1 kernel driver compilation for LLM-8850, wired up Module Audio M144 with I2S, and solved I2C address conflicts for 3 encoders using PaHub v2.1.
- Use flair: `Tutorial` or `Technical Discussion`.

## For X (Twitter)

**Thread Hook (First Tweet):**

> Spent the week debugging some gnarly M5Stack issues 🤯  
>   
> From button callbacks on Fire v2.7 to Linux 6.1 kernel driver nightmares, here's what I learned. 🧵👇  
>   
> #M5Stack #ESP32 #IoT #EmbeddedSystems #MicroPython

**Follow-up tweets:**
- Tweet 2: "1️⃣ Fire v2.7 button events failing? Likely using generic MicroPython instead of UiFlow2 firmware. Here's the M5Unified fix: [code snippet screenshot]"
- Tweet 3: "2️⃣ LLM-8850 driver won't compile on Linux 6.1? Official prebuilt only supports 5.15/6.8. Manual compilation from AX SDK saved the day. [link to docs]"
- Tweet 4: "3️⃣ Wiring Module Audio M144 to Basic v2.7? I2C+I2S config was the key. Full example with mic passthrough: [code snippet screenshot]"
- Tweet 5: "4️⃣ Need multiple encoders with the same I2C address? PaHub v2.1 is a lifesaver. Channel isolation = no more conflicts. Perfect for Matter lighting. 💡"
- Tweet 6: "Full breakdown with code examples: [link to blog/forum post] 📝"

---

**End of Article**