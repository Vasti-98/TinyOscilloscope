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
zoom_value = 4  # Default zoom (1x)

# ========== GET ZOOM FACTOR (Option 4: from variable) ==========
def get_zoom_factor():
    """
    Replace this function to read from a potentiometer or ADC input.
    For now, manually adjust `zoom_value` below for testing.

    Later, replace get_zoom_factor() with a value from a potentiometer input (e.g., analogRead() mapped to 0.5–4.0 range from your microcontroller).
    """
    return zoom_value



# ========== SIMULATION ==========
def simulate_packet():
    """Generates a pulse waveform and packs it into a simulated packet."""
    global sim_phase
    t = np.linspace(0, 1, SAMPLES)

    # Create pulse waveform
    freq = 5
    pulse_wave = (np.sin(2 * np.pi * freq * t + sim_phase) > 0).astype(np.int16)
    amplitude = 30000
    waveform = pulse_wave * amplitude

    noise = np.random.normal(0, 500, SAMPLES)
    waveform = waveform + noise
    waveform = np.clip(waveform, 0, 32767).astype(np.int16)

    sim_phase += 0.2

    adc_bytes = b''.join(struct.pack('<H', val) for val in waveform)
    header = bytes([0xAA, 0x55])
    gain_bytes = struct.pack('<H', np.random.randint(1, 20))
    trigger_bytes = struct.pack('<H', np.random.randint(2000, 3000))

    return header + adc_bytes + gain_bytes + trigger_bytes

# ========== DATA READER ==========
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
line, = ax.plot(np.zeros(SAMPLES))
ax.set_ylim(0, 10)
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Voltage (V)")
ax.set_title("Real-Time Pulse Waveform")

# ========== MAIN LOOP ==========
while True:
    with data_lock:
        adc_data = global_data['adc']
        gain = global_data['gain']
        trigger = global_data['trigger']

    if adc_data is not None:
        voltage_data = (adc_data.astype(np.float64) / 32767.0) * 10.0

        # Get dynamic zoom factor
        zoom_factor = get_zoom_factor()
        visible_samples = int(SAMPLES / zoom_factor)
        visible_samples = max(10, min(visible_samples, SAMPLES))

        time_axis = np.arange(visible_samples) * (1000.0 / sample_rate_hz)
        voltage_view = voltage_data[:visible_samples]

        line.set_data(time_axis, voltage_view)
        ax.set_xlim(time_axis[0], time_axis[-1])
        fig.canvas.draw()
        fig.canvas.flush_events()

        print(f"Gain: {gain}, Trigger: {trigger}, Zoom: {zoom_factor:.2f}x")

    plt.pause(0.1)
