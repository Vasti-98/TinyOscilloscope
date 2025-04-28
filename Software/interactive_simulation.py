# Required: pip install numpy matplotlib

import threading
import numpy as np
import matplotlib.pyplot as plt
import struct
import time

# ========== CONFIG ==========
USE_SIMULATION = True
sample_rate_hz = 10000
serial_port = "COM3"
baud_rate = 115200

PACKET_SIZE = 2054
SAMPLES = 1024
zoom_value = 2.0  # Initial default zoom
manual_trigger_level = 1.7  # in Volts

# ========== GLOBAL STATE ==========
global_data = {
    'adc': None,
    'gain': None,
    'trigger': None
}
data_lock = threading.Lock()
sim_phase = 0

# ========== ZOOM LOGIC ==========
def get_zoom_factor():
    return zoom_value

# ========== SIMULATED PACKET GENERATOR ==========
def simulate_packet():
    global sim_phase, zoom_value
    t = np.linspace(0, 1, SAMPLES)
    freq = 5
    pulse = (np.sin(2 * np.pi * freq * t + sim_phase) > 0).astype(np.int16)
    amplitude = 28000
    noise = np.random.normal(0, 500, SAMPLES)

    simulated_zoom_adc = 2700
    zoom_voltage = (simulated_zoom_adc / 4095.0) * 3.3
    zoom_mapped = np.interp(zoom_voltage, [1.7, 3.3], [0.5, 5.0])
    zoom_value = zoom_mapped

    offset_voltage = np.random.choice([2.0, 4.0, 6.0, 8.0])
    offset_adc = int((offset_voltage / 10.0) * 32767)
    waveform = pulse * amplitude + offset_adc + noise
    waveform = np.clip(waveform, 0, 32767).astype(np.int16)

    sim_phase += 0.3

    adc_bytes = b''.join(struct.pack('<H', val) for val in waveform)
    header = bytes([0xAA, 0x55])
    gain_bytes = struct.pack('<H', np.random.randint(1, 10))
    trigger_bytes = struct.pack('<H', 0)  # Unused now

    return header + adc_bytes + gain_bytes + trigger_bytes

# ========== READ PACKETS ==========
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

        if packet[0] != 0xAA or packet[1] != 0x55:
            continue

        adc_bytes = packet[2:2050]
        adc_data = np.frombuffer(adc_bytes, dtype='<H')
        gain = struct.unpack('<H', packet[2050:2052])[0]

        with data_lock:
            global_data['adc'] = adc_data
            global_data['gain'] = gain

# ========== START READER THREAD ==========
threading.Thread(target=read_serial_packets, daemon=True).start()

# ========== PLOTTING ==========
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(np.zeros(SAMPLES), label="Signal")
trigger_line = ax.axhline(manual_trigger_level, color='r', linestyle='--', label='Trigger Level')
ax.set_ylim(0, 10)
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Voltage (V)")
ax.set_title("Real-Time Pulse with Trigger")
ax.legend()

# ========== KEYBOARD SHORTCUT ==========
def on_key(event):
    global manual_trigger_level
    if event.key == 'up':
        manual_trigger_level = min(manual_trigger_level + 0.1, 10.0)
        print(f"Trigger level increased to {manual_trigger_level:.2f} V")
    elif event.key == 'down':
        manual_trigger_level = max(manual_trigger_level - 0.1, 0.0)
        print(f"Trigger level decreased to {manual_trigger_level:.2f} V")
    trigger_line.set_ydata([manual_trigger_level, manual_trigger_level])
    fig.canvas.draw()

fig.canvas.mpl_connect('key_press_event', on_key)

# ========== MAIN LOOP ==========
while True:
    with data_lock:
        adc_data = global_data['adc']
        gain = global_data['gain']

    if adc_data is not None:
        voltage_data = (adc_data.astype(np.float64) / 32767.0) * 10.0
        trigger_voltage = manual_trigger_level

        trigger_index = None
        for i in range(1, len(voltage_data)):
            if voltage_data[i - 1] < trigger_voltage <= voltage_data[i]:
                trigger_index = i
                break

        if trigger_index is not None:
            zoom_factor = get_zoom_factor()
            visible_samples = int(SAMPLES / zoom_factor)
            visible_samples = max(10, min(visible_samples, SAMPLES))

            end_index = trigger_index + visible_samples
            if end_index > SAMPLES:
                end_index = SAMPLES
                trigger_index = end_index - visible_samples

            time_axis = np.arange(visible_samples) * (1000.0 / sample_rate_hz)
            voltage_view = voltage_data[trigger_index:end_index]

            line.set_data(time_axis, voltage_view)
            trigger_line.set_ydata([trigger_voltage, trigger_voltage])
            ax.set_xlim(time_axis[0], time_axis[-1])

            fig.canvas.draw()
            fig.canvas.flush_events()
            #print(f"Gain: {gain}, Trigger: {trigger_voltage:.2f} V, Zoom: {zoom_factor:.2f}x")
            print(voltage_data)
            print(f"Trigger: {trigger_index:.2f}")
        else:
            #print("Waiting for trigger...")
            print()

    plt.pause(0.1)
