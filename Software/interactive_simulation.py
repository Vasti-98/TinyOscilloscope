# Required: pip install numpy matplotlib

import threading
import numpy as np
import matplotlib.pyplot as plt
import struct
import time

# ========== CONFIG ==========

USE_SIMULATION = True
sample_rate_hz = 10000  # Simulated sample rate: 10kHz
serial_port = "COM3"    # Ignored in simulation mode
baud_rate = 115200

PACKET_SIZE = 2054  # 2 header + 2048 ADC + 2 gain + 2 trigger
SAMPLES = 1024

# ========== GLOBALS ==========

global_data = {
    'adc': None,
    'gain': None,
    'trigger': None
}
data_lock = threading.Lock()

sim_phase = 0
zoom_value = 1  # Default zoom (1x)

last_good_voltage_view = None
last_good_time_axis = None

# ========== GET ZOOM FACTOR (Option 4: from variable) ==========

def get_zoom_factor():
    return zoom_value  # Later can be updated from potentiometer input

# ========== SIMULATION ==========

def simulate_packet():
    """Generates a pulse waveform and packs it into a simulated packet.If you want to simulate 
    rising/falling edges in the future, you could track if the trigger goes from 0 → 1 (rising) or 
    1 → 0 (falling), but for now simple trigger == 1 detection is enough!"""

    global sim_phase
    t = np.linspace(0, 1, SAMPLES)

    # Create pulse waveform
    freq = 5  # 5Hz
    pulse_wave = (np.sin(2 * np.pi * freq * t + sim_phase) > 0).astype(np.int16)
    amplitude = 30000
    waveform = pulse_wave * amplitude

    noise =0 
    #np.random.normal(0, 500, SAMPLES)
    waveform = waveform + noise
    waveform = np.clip(waveform, 0, 1023).astype(np.int16)

        # Simulate gain: Random gain between 1 and 20
    gain = 2

    # Apply gain to waveform (simulating the effect of the AGC)
    waveform = (waveform * gain) / 10  # Apply gain scaling (with attenuation for simplicity)

    # Ensure we don't go out of ADC range (0 to 1023)
    waveform = np.clip(waveform, 0, 1023).astype(np.int16)
    
    adc_bytes = b''.join(struct.pack('<H', val) for val in waveform)
    header = bytes([0xAA, 0x55])
    gain_bytes = struct.pack('<H', gain)
    trigger_bytes = struct.pack('<H', 1)  # Simulate digital TRIG = 1

    return header + adc_bytes + gain_bytes + trigger_bytes

# ========== DATA READER ==========s

def read_serial_packets():
    while True:
        if USE_SIMULATION:
            packet = simulate_packet()
            time.sleep(0.1)
        else:
            import serial
            try:
                with serial.Serial(serial_port, baud_rate, timeout=1) as ser:
                    packet = ser.read(PACKET_SIZE)
                    if len(packet) < PACKET_SIZE:
                        continue
            except serial.SerialException as e:
                print("Serial error:", e)
                continue

        # Parse packet
        if packet[0] != 0xAA or packet[1] != 0x55:
            print("Header mismatch, skipping packet.")
            continue

        adc_bytes = packet[2:2050]
        adc_data = np.frombuffer(adc_bytes, dtype='<H')

        gain = struct.unpack('<H', packet[2050:2052])[0]
        trigger = struct.unpack('<H', packet[2052:2054])[0]

        with data_lock:
            global_data['adc'] = adc_data
            global_data['gain'] = gain
            global_data['trigger'] = trigger

# ========== START THREAD ==========

reader_thread = threading.Thread(target=read_serial_packets, daemon=True)
reader_thread.start()

# ========== PLOTTING ==========

plt.ion()
fig, ax = plt.subplots()
#trigger_line = ax.axhline(manual_trigger_level, color='r', linestyle='--', label='Trigger Level')
line, = ax.plot(np.zeros(SAMPLES))
ax.set_ylim(0, 10)  # 0-10V
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Voltage (V)")
ax.set_title("Real-Time Pulse Waveform with Hardware Triggering")

info_text = ax.text(
    0.02, 0.95, '', transform=ax.transAxes,
    verticalalignment='top', bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    fontsize=10
)
# ========== MAIN LOOP ==========

triggered = False


while True:
    with data_lock:
            # Assuming global_data contains adc_data, gain, trigger, and other parameters.
        adc_data = global_data['adc']
        gain = global_data['gain']  # This is assumed to be the AGC gain value
        trigger = global_data['trigger']

    if adc_data is not None: #need to edit the zoom factor need to consider the gain. 

        #NEED TO FIX THIS 
        # Step 1: Convert ADC data to measured voltage (0–3.3V)
        v_measured = (adc_data.astype(np.float64) / 1023.0) * 1.0  # Assuming 10-bit ADC, VDDA = 3.3V
        
        # Step 2: Use the 'gain' (which is the AGC gain measured from another ADC pin)
        # If 'gain' represents VGAIN, which is scaled as 20mV/dB, convert it to linear gain:
        agc_gain = 10 ** (gain / 20.0)  # Convert gain from dB to linear scale
        
        # Step 3: Reconstruct the original input voltage (scaled by 10 because of the attenuator)
        voltage_data = (v_measured / agc_gain) * 10.0  # Scaling the voltage back to the original range

        # Step 4: Apply zoom factor and determine the visible sample range for display
        zoom_factor = get_zoom_factor()  # Vertical adjustment
        visible_samples = int(SAMPLES / zoom_factor)
        visible_samples = max(10, min(visible_samples, SAMPLES))  # Ensure visible_samples are within valid range

        # Step 5: Update the info text with the latest variables (like gain, trigger, and zoom)
        info_text.set_text(
            f"Gain: {gain}\nTrigger: {trigger}\nZoom: {zoom_factor:.2f}x"
        )

        # === Hardware trigger detection ===
        if trigger == 1:  # Hardware trigger asserted
            print("Hardware Trigger Detected!")

            trigger_index = 0  # You can define trigger windowing if needed

            end_index = trigger_index + visible_samples
            if end_index > len(voltage_data):
                end_index = len(voltage_data)
                trigger_index = end_index - visible_samples

            time_axis = np.arange(visible_samples) * (1000.0 / sample_rate_hz)
            voltage_view = voltage_data[trigger_index:trigger_index + visible_samples]

            # Update the plot
            line.set_data(time_axis, voltage_view)
            ax.set_xlim(time_axis[0], time_axis[-1])
            fig.canvas.draw()
            fig.canvas.flush_events()

            # Save last good waveform
            last_good_voltage_view = voltage_view.copy()
            last_good_time_axis = time_axis.copy()
            sim_phase = 0.0
            triggered = True

        else:
            """
            if triggered and last_good_voltage_view is not None:
                # Show last good frame
                line.set_data(last_good_time_axis, last_good_voltage_view)
                ax.set_xlim(last_good_time_axis[0], last_good_time_axis[-1])
                fig.canvas.draw()
                fig.canvas.flush_events() """
            print("NOT TRIGGERED")

        # Print useful debug info
        print(f"Gain: {gain}, Trigger: {trigger}, Zoom: {zoom_factor:.2f}x")

    plt.pause(0.05)
