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
time_axis = np.arange(SAMPLES) * (1000.0 / sample_rate_hz)  # milliseconds

# ========== GLOBALS ==========
global_data = {
    'adc': None,
    'gain': None,
    'trigger': None
}
data_lock = threading.Lock()
sim_phase = 0

# ========== SIMULATION ==========
def simulate_packet():
    """Generates a pulse waveform and packs it into a simulated packet."""
    global sim_phase
    t = np.linspace(0, 1, SAMPLES)

    # Create a pulse wave by thresholding a sine wave
    freq = 5  # Number of pulses
    pulse_wave = (np.sin(2 * np.pi * freq * t + sim_phase) > 0).astype(np.int16)

    amplitude = 30000
    waveform = pulse_wave * amplitude

    # Add light noise
    noise = np.random.normal(0, 500, SAMPLES)
    waveform = waveform + noise
    waveform = np.clip(waveform, 0, 32767).astype(np.int16)

    sim_phase += 0.2  # Advance phase to animate

    adc_bytes = b''.join(struct.pack('<H', val) for val in waveform)  # unsigned
    header = bytes([0xAA, 0x55])
    gain_bytes = struct.pack('<H', np.random.randint(1, 20))
    trigger_bytes = struct.pack('<H', np.random.randint(2000, 3000))

    return header + adc_bytes + gain_bytes + trigger_bytes

# ========== DATA READER ==========
def read_serial_packets():
    """Reads either simulated or real packets."""
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

        # Parse header
        if packet[0] != 0xAA or packet[1] != 0x55:
            print("Header mismatch, skipping packet.")
            continue

        adc_bytes = packet[2:2050]
        adc_data = np.frombuffer(adc_bytes, dtype='<H')  # uint16 little-endian

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
line, = ax.plot(time_axis, np.zeros(SAMPLES))
ax.set_xlim(time_axis[0], time_axis[-1])
ax.set_ylim(0, 10)  # Voltage in V
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Voltage (V)")
ax.set_title("Real-Time Pulse Waveform (Simulated)")

# ========== MAIN LOOP ==========
while True:
    with data_lock:
        adc_data = global_data['adc']
        gain = global_data['gain']
        trigger = global_data['trigger']

    if adc_data is not None:
        voltage_data = (adc_data.astype(np.float64) / 32767.0) * 10.0
        line.set_ydata(voltage_data)
        fig.canvas.draw()
        fig.canvas.flush_events()
        print(f"Gain: {gain}, Trigger: {trigger}")

    plt.pause(0.1)
