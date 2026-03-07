import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import queue
import time
import os
import binascii

# =============================================================================
# CONSTANTS & CONFIG
# =============================================================================
BAUD_RATE = 9600
EEPROM_SIZE = 131072
CMD_TIMEOUT = 2.0 
# Optimized delay: EEPROM write is max 5ms. 
# Transmission of 2 chars at 9600 baud takes ~2.1ms.
# We wait 4.0ms, relying on the AVR single-byte UART buffer to catch 
# the next start bit while the write cycle completes.
WRITE_DELAY = 0.0040 

# =============================================================================
# SERIAL WORKER
# =============================================================================
class SerialWorker:
    def __init__(self, ui_queue):
        self.ser = None
        self.ui_queue = ui_queue
        self.running = False
        self.write_queue = queue.Queue()
        
        # Synchronization Events
        self.evt_ack = threading.Event()      # For "OK" / "Error"
        self.evt_ready = threading.Event()    # For "READY"
        self.evt_chunk_ack = threading.Event() # For ">" (Chunk ACK)
        
        self.upload_active = False

    def connect(self, port):
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
            # Opening the port toggles DTR which triggers an AVR reset.
            # Wait briefly for the bootloader to finish, then discard any
            # stray bytes (e.g. a bare '\r') so the first RX line is clean.
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.running = True
            threading.Thread(target=self._reader_thread, daemon=True).start()
            threading.Thread(target=self._writer_thread, daemon=True).start()
            return True, "Connected"
        except serial.SerialException as e:
            return False, str(e)

    def disconnect(self):
        self.running = False
        self.upload_active = False
        if self.ser:
            try: self.ser.close()
            except: pass
            self.ser = None

    def send(self, cmd):
        if self.running:
            self.write_queue.put(cmd)

    def _writer_thread(self):
        while self.running:
            try:
                cmd = self.write_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self.upload_active: continue

            if self.ser and self.ser.is_open:
                self.evt_ack.clear()
                full_cmd = cmd.strip() + '\r'
                try:
                    self.ser.write(full_cmd.encode('ascii'))
                    self.ui_queue.put(("TX", cmd.strip()))
                except Exception as e:
                    self.ui_queue.put(("ERR", f"Write: {e}"))
                    continue

                if not self.evt_ack.wait(CMD_TIMEOUT):
                    pass 
                else:
                    time.sleep(0.05) 

    def _reader_thread(self):
        buffer = ""
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    raw_data = self.ser.read(self.ser.in_waiting)
                    decoded_chunk = raw_data.decode('ascii', errors='ignore')

                    # 1. HANDLE CHUNK ACK ('>')
                    # We check for '>' which signals the board is ready for the next chunk
                    if self.upload_active and '>' in decoded_chunk:
                        self.evt_chunk_ack.set()
                        # Remove '>' so it doesn't mess up text processing
                        decoded_chunk = decoded_chunk.replace('>', '')

                    buffer += decoded_chunk

                    # 2. PROCESS FULL LINES
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        # Only strip CR/LF — preserve leading spaces (indentation).
                        # Then drop non-printable chars (e.g. \x00 after AVR reset)
                        # while keeping spaces so indentation is not lost.
                        line = line.strip('\r\n')
                        line = ''.join(c for c in line if c == ' ' or c.isprintable())
                        if not line: continue
                        
                        if self.upload_active:
                            if "READY" in line:
                                self.evt_ready.set()
                            if "OK" in line:
                                self.evt_ack.set()

                        self.ui_queue.put(("RX", line))
                        
                        if not self.upload_active:
                            if line == "OK" or line.startswith("Error"):
                                self.evt_ack.set()
                                
            except Exception as e:
                self.ui_queue.put(("ERR", f"Reader: {e}"))
                break
            time.sleep(0.005)

    def upload_eeprom(self, raw_data):
        threading.Thread(target=self._upload_task, args=(raw_data,), daemon=True).start()

    def _upload_task(self, data):
        if len(data) > EEPROM_SIZE:
            self.ui_queue.put(("ERR", f"Data too large! {len(data)} > {EEPROM_SIZE}"))
            return

        CHUNK_SIZE = 32
        self.upload_active = True
        
        with self.write_queue.mutex:
            self.write_queue.queue.clear()
        self.evt_ready.clear()
        self.evt_ack.clear()

        try:
            self.ui_queue.put(("STATUS", "Starting upload..."))
            self.ser.reset_input_buffer()
            self.ser.write(b"AT+upload\r")
            self.ui_queue.put(("TX", "AT+upload"))
            
            if not self.evt_ready.wait(4.0):
                raise Exception("Timeout: Board did not send READY")

            time.sleep(0.2)
            self.ui_queue.put(("SYS", "Board Ready. Sending chunks..."))

            pad_len = CHUNK_SIZE - (len(data) % CHUNK_SIZE)
            if pad_len != CHUNK_SIZE:
                data += b'\xFF' * pad_len

            total_bytes = len(data)
            hex_data = data.hex().upper()
            total_chunks = len(hex_data) // (CHUNK_SIZE * 2)
            chunks_done = 0
            
            for i in range(0, len(hex_data), CHUNK_SIZE * 2):
                if not self.running: break
                
                chunk_hex = hex_data[i : i + (CHUNK_SIZE * 2)]
                
                self.evt_chunk_ack.clear()
                
                self.ser.write(chunk_hex.encode('ascii'))
                self.ser.flush()
                
                # Wait for '>'
                if not self.evt_chunk_ack.wait(5.0):
                    raise Exception(f"Timeout waiting for '>' at offset {i//2}")

                chunks_done += 1
                percent = (chunks_done / total_chunks) * 100
                self.ui_queue.put(("PROGRESS", percent))

            # Wait for final OK — firmware needs ~1.5 s idle timeout + ~80 ms padding writes
            if not self.evt_ack.wait(30.0):
                self.ui_queue.put(("WARN", "Final OK timed out"))

            self.ui_queue.put(("PROGRESS", 100))
            self.ui_queue.put(("STATUS", "Upload Complete"))

        except Exception as e:
            self.ui_queue.put(("ERR", f"Upload failed: {e}"))
            try: self.ser.write(b'\x1B')
            except: pass
        finally:
            self.upload_active = False
            # Reset progress bar to 0 after 2 seconds so it doesn't look stuck
            time.sleep(2.0)
            self.ui_queue.put(("PROGRESS", 0))

# =============================================================================
# MAIN GUI
# =============================================================================
class RttyControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RTTY/AM Transmitter Control Center")
        self.root.geometry("1100x650")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.queue = queue.Queue()
        self.worker = SerialWorker(self.queue)
        self.connected = False
        self.pcm_data = None
        self._upload_start_time = None
        self._upload_total_bytes = 0

        style = ttk.Style()
        style.theme_use('clam')
        self.create_ui()
        self.process_queue()

    def create_ui(self):
        self.root.columnconfigure(0, weight=0)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # === LEFT PANEL ===
        left_panel = ttk.Frame(self.root, padding=10)
        left_panel.grid(row=0, column=0, sticky="nsew")
        
        # Connection
        conn_frame = ttk.Frame(left_panel)
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(conn_frame, text="Port:").pack(side=tk.LEFT)
        self.cb_ports = ttk.Combobox(conn_frame, width=15)
        self.cb_ports.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()
        ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports).pack(side=tk.LEFT)
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side=tk.LEFT, padx=10)
        
        # Fixed width status label to prevent jitter
        self.lbl_status = ttk.Label(conn_frame, text="Disconnected", foreground="red", width=25)
        self.lbl_status.pack(side=tk.LEFT, padx=5)

        self.notebook = ttk.Notebook(left_panel)
        self.notebook.pack(expand=True, fill=tk.BOTH)

        # Tabs
        self.tab_rtty = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_rtty, text="RTTY Transmission")
        self.build_rtty_tab()

        self.tab_audio = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_audio, text="Audio / EEPROM")
        self.build_audio_tab()

        self.tab_gpio = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_gpio, text="GPIO Control")
        self.build_gpio_tab()

        self.tab_sys = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_sys, text="System")
        self.build_sys_tab()

        # === RIGHT PANEL (LOGS) ===
        right_panel = ttk.Frame(self.root, padding=10)
        right_panel.grid(row=0, column=1, sticky="nsew")

        log_toolbar = ttk.Frame(right_panel)
        log_toolbar.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(log_toolbar, text="Command Log", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        ttk.Button(log_toolbar, text="Clear Log", command=self.clear_log).pack(side=tk.RIGHT, padx=5)
        ttk.Button(log_toolbar, text="Save Log", command=self.save_log).pack(side=tk.RIGHT)

        self.log_text = scrolledtext.ScrolledText(right_panel, state='disabled', font=("Consolas", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.tag_config("TX", foreground="blue")
        self.log_text.tag_config("RX", foreground="green")
        self.log_text.tag_config("ERR", foreground="red")
        self.log_text.tag_config("SYS", foreground="gray")
        self.log_text.tag_config("WARN", foreground="orange")

        entry_frame = ttk.Frame(right_panel, padding=(0, 5, 0, 0))
        entry_frame.pack(fill=tk.X)
        ttk.Label(entry_frame, text="Manual CMD:").pack(side=tk.LEFT)
        self.ent_manual = ttk.Entry(entry_frame)
        self.ent_manual.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.ent_manual.bind("<Return>", lambda e: self.send_manual())
        ttk.Button(entry_frame, text="Send", command=self.send_manual).pack(side=tk.LEFT)

    def build_rtty_tab(self):
        pad = {'padx': 10, 'pady': 10}
        frame_msg = ttk.LabelFrame(self.tab_rtty, text="Transmit Message", padding=10)
        frame_msg.pack(fill=tk.X, **pad)
        self.txt_rtty_msg = tk.Text(frame_msg, height=4, width=50)
        self.txt_rtty_msg.pack(fill=tk.X, pady=5)
        self.txt_rtty_msg.insert("1.0", "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG 1234567890")
        ttk.Button(frame_msg, text="TRANSMIT MESSAGE", command=self.send_rtty_msg).pack(anchor='e')

        frame_set = ttk.LabelFrame(self.tab_rtty, text="Current Parameters", padding=10)
        frame_set.pack(fill=tk.X, **pad)
        cols = ["Mark (High)", "Space (Low)", "Baud"]
        self.vars_rtty = {}
        for i, col in enumerate(cols):
            ttk.Label(frame_set, text=col).grid(row=0, column=i*2, sticky='w')
            var = tk.StringVar()
            entry = ttk.Entry(frame_set, textvariable=var, width=10)
            entry.grid(row=0, column=i*2+1, padx=5, sticky='w')
            self.vars_rtty[col] = var
        btn_frame = ttk.Frame(frame_set)
        btn_frame.grid(row=1, column=0, columnspan=6, pady=10)
        ttk.Button(btn_frame, text="Read from Board", command=self.get_rtty_params).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Apply to Board", command=self.set_rtty_params).pack(side=tk.LEFT, padx=5)

    def build_audio_tab(self):
        pad = {'padx': 10, 'pady': 10}
        frame_play = ttk.LabelFrame(self.tab_audio, text="On-Board Playback", padding=10)
        frame_play.pack(fill=tk.X, **pad)
        ttk.Label(frame_play, text="Plays audio stored in EEPROM.").pack(anchor='w')
        ttk.Button(frame_play, text="PLAY AUDIO (AT+play)", command=lambda: self.worker.send("AT+play")).pack(pady=5)

        frame_up = ttk.LabelFrame(self.tab_audio, text="Upload New Audio", padding=10)
        frame_up.pack(fill=tk.BOTH, expand=True, **pad)

        # Make column 1 expand so controls reach the right edge
        frame_up.columnconfigure(0, weight=0)
        frame_up.columnconfigure(1, weight=1)
        frame_up.columnconfigure(2, weight=0)

        # Row 0: file picker
        ttk.Label(frame_up, text="Selected File:").grid(row=0, column=0, sticky='w')
        self.lbl_file = ttk.Label(frame_up, text="None", foreground="gray")
        self.lbl_file.grid(row=0, column=1, sticky='ew', padx=5)
        ttk.Button(frame_up, text="Browse...", command=self.browse_audio).grid(row=0, column=2, sticky='e')

        # Row 1: file info (size + estimated upload time)
        self.lbl_file_info = ttk.Label(frame_up, text="")
        self.lbl_file_info.grid(row=1, column=0, columnspan=3, sticky='w', pady=5)

        # Row 2: separator — sticky='ew' so it spans full width
        ttk.Separator(frame_up, orient='horizontal').grid(row=2, column=0, columnspan=3, sticky='ew', pady=10)

        # Row 3: upload button
        self.btn_upload = ttk.Button(frame_up, text="Upload to EEPROM", state='disabled', command=self.start_upload)
        self.btn_upload.grid(row=3, column=0, columnspan=3, sticky='ew')

        # Row 4: progress bar (full width) with percentage label on the right
        pb_frame = ttk.Frame(frame_up)
        pb_frame.grid(row=4, column=0, columnspan=3, sticky='ew', pady=(10, 2))
        pb_frame.columnconfigure(0, weight=1)
        self.pb_upload = ttk.Progressbar(pb_frame, orient='horizontal', mode='determinate')
        self.pb_upload.grid(row=0, column=0, sticky='ew')
        self.lbl_pb_pct = ttk.Label(pb_frame, text="0%", width=5, anchor='e')
        self.lbl_pb_pct.grid(row=0, column=1, padx=(6, 0))

        # Row 5: ETA label
        self.lbl_eta = ttk.Label(frame_up, text="", foreground="gray")
        self.lbl_eta.grid(row=5, column=0, columnspan=3, sticky='w')

    def build_gpio_tab(self):
        frame = ttk.Frame(self.tab_gpio, padding=20)
        frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(frame, text="Direct GPIO Control (PD2 - PD7)", font=("Arial", 10, "bold")).pack(pady=(0,20))
        grid_frame = ttk.Frame(frame)
        grid_frame.pack()
        for pin in range(6):
            p_frame = ttk.Frame(grid_frame, borderwidth=1, relief="sunken", padding=10)
            p_frame.grid(row=pin//3, column=pin%3, padx=10, pady=10)
            lbl = ttk.Label(p_frame, text=f"GPIO {pin}\n(PD{pin+2})", justify='center')
            lbl.pack()
            ttk.Button(p_frame, text="HIGH", command=lambda p=pin: self.worker.send(f"AT+write={p},1")).pack(pady=2)
            ttk.Button(p_frame, text="LOW", command=lambda p=pin: self.worker.send(f"AT+write={p},0")).pack(pady=2)

    def build_sys_tab(self):
        frame = ttk.Frame(self.tab_sys, padding=20)
        frame.pack()
        ttk.Button(frame, text="Check Connection (AT)", command=lambda: self.worker.send("AT")).pack(fill=tk.X, pady=5)
        ttk.Button(frame, text="Factory Reset (AT+reset)", command=lambda: self.worker.send("AT+reset")).pack(fill=tk.X, pady=5)
        ttk.Button(frame, text="Help (AT+help)", command=lambda: self.worker.send("AT+help")).pack(fill=tk.X, pady=5)

    # --- LOGIC ---
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.cb_ports['values'] = [p.device for p in ports]
        if ports: self.cb_ports.current(0)

    def toggle_connection(self):
        if not self.connected:
            port = self.cb_ports.get()
            if not port: return
            success, msg = self.worker.connect(port)
            if success:
                self.connected = True
                self.btn_connect.config(text="Disconnect")
                self.lbl_status.config(text="Connected", foreground="green")
                self.log("SYS", f"Connected to {port}")
                self.worker.send("AT") 
                self.get_rtty_params()
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            self.worker.disconnect()
            self.connected = False
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="Disconnected", foreground="red")
            self.log("SYS", "Disconnected")

    def browse_audio(self):
        filename = filedialog.askopenfilename(filetypes=[("All Files", "*.*"), ("Text Files", "*.txt"), ("WAV Files", "*.wav")])
        if filename:
            self.lbl_file.config(text=os.path.basename(filename), foreground="black")
            try:
                self.log("SYS", f"Loading {os.path.basename(filename)}...")
                
                with open(filename, "rb") as f:
                    raw_content = f.read()

                # --- AUTO-DETECTION LOGIC ---
                is_wav = raw_content.startswith(b'RIFF')
                is_hex_text = False
                
                # Check if it's text (hex dump)
                try:
                    text_content = raw_content.decode('ascii').strip()
                    # If strictly hex characters and spaces/newlines
                    if all(c in '0123456789ABCDEFabcdef \r\n' for c in text_content):
                        is_hex_text = True
                except:
                    pass

                final_data = raw_content

                if is_wav:
                    self.log("WARN", "WAV header detected! This will be uploaded as raw noise.")
                    msg = f"WAV Header detected.\n\nType: WAV File\nSize: {len(raw_content)} bytes\n\nUpload as-is?"
                    if not messagebox.askyesno("WAV Detected", msg):
                        self.pcm_data = None
                        self.btn_upload.state(['disabled'])
                        return

                elif is_hex_text:
                    self.log("SYS", "Hex-Dump Text detected. Converting to Binary...")
                    # Clean and convert
                    clean_hex = text_content.replace(' ', '').replace('\r', '').replace('\n', '')
                    try:
                        final_data = binascii.unhexlify(clean_hex)
                        self.log("SYS", f"Converted Text to {len(final_data)} binary bytes.")
                    except Exception as e:
                        self.log("ERR", f"Conversion failed: {e}")
                        messagebox.showerror("Error", "Could not convert hex text to binary.")
                        return

                # Check Size
                size = len(final_data)

                # Estimate upload time:
                # Each byte → 2 hex chars → 2 chars × (10 bits / 9600 baud) ≈ 2.08 ms TX
                # Plus WRITE_DELAY (4 ms) per byte, plus EEPROM page write ~5 ms per 32 bytes
                # Dominant cost: page writes. 1 page = 32 bytes written in ~5 ms.
                # Also ~1.5 s idle timeout at the end + 16 padding page writes (~80 ms).
                num_pages = (size + 31) // 32
                tx_ms     = size * (2 * 10 / 9600) * 1000   # serial TX time
                write_ms  = num_pages * 5                    # EEPROM page write time
                idle_ms   = 1500                             # firmware idle timeout
                pad_ms    = 16 * 5                           # 16 padding pages
                total_sec = (tx_ms + write_ms + idle_ms + pad_ms) / 1000.0

                if total_sec < 60:
                    eta_str = f"~{total_sec:.0f} s"
                else:
                    eta_str = f"~{total_sec/60:.1f} min"

                info = f"{size:,} bytes  •  Est. upload time: {eta_str}"
                self.lbl_file_info.config(text=info)

                if size > EEPROM_SIZE:
                    self.lbl_file_info.config(text=info + "\nERROR: > 128KB!", foreground="red")
                    self.btn_upload.state(['disabled'])
                    self.pcm_data = None
                else:
                    self.lbl_file_info.config(foreground="black")
                    self.btn_upload.state(['!disabled'])
                    self.pcm_data = final_data
                    
                    # Preview first few bytes
                    preview = final_data[:8].hex().upper()
                    self.log("SYS", f"First 8 bytes: {preview}")

            except Exception as e:
                messagebox.showerror("File Error", str(e))

    def start_upload(self):
        if not self.connected: return
        if self.pcm_data:
            self._upload_start_time = None   # reset; will be set on first PROGRESS
            self._upload_total_bytes = len(self.pcm_data)
            self.lbl_eta.config(text="")
            self.lbl_pb_pct.config(text="0%")
            self.btn_upload.state(['disabled'])
            self.worker.upload_eeprom(self.pcm_data)

    def send_rtty_msg(self):
        msg = self.txt_rtty_msg.get("1.0", tk.END).strip()
        if msg:
            msg = msg.replace('"', '') 
            self.worker.send(f'AT+send="{msg}"')

    def get_rtty_params(self):
        self.worker.send("AT+get=high")
        self.worker.send("AT+get=low")
        self.worker.send("AT+get=baud")

    def set_rtty_params(self):
        try:
            h = self.vars_rtty["Mark (High)"].get()
            l = self.vars_rtty["Space (Low)"].get()
            b = self.vars_rtty["Baud"].get()
            if h: self.worker.send(f"AT+set=high,{h}")
            if l: self.worker.send(f"AT+set=low,{l}")
            if b: self.worker.send(f"AT+set=baud,{b}")
            self.root.after(500, self.get_rtty_params)
        except: pass

    def send_manual(self):
        cmd = self.ent_manual.get()
        if cmd:
            self.worker.send(cmd)
            self.ent_manual.delete(0, tk.END)

    def log(self, tag, msg):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"[{timestamp}] [{tag}] {msg}\n", tag)
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

    def clear_log(self):
        self.log_text.config(state='normal')
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state='disabled')

    def save_log(self):
        fn = filedialog.asksaveasfilename(defaultextension=".log",
                                          filetypes=[("Log Files", "*.log")])
        if fn:
            try:
                with open(fn, 'w') as f:
                    f.write(self.log_text.get(1.0, tk.END))
                self.log("SYS", f"Log saved to {fn}")
            except Exception as e:
                messagebox.showerror("Save Error", str(e))

    def process_queue(self):
        try:
            while True:
                tag, msg = self.queue.get_nowait()
                if tag == "PROGRESS":
                    pct = float(msg)
                    self.pb_upload['value'] = pct
                    self.lbl_pb_pct.config(text=f"{pct:.0f}%")

                    if pct > 0 and pct < 100:
                        if self._upload_start_time is None:
                            self._upload_start_time = time.time()
                        else:
                            elapsed = time.time() - self._upload_start_time
                            if elapsed > 0:
                                remaining = elapsed / (pct / 100.0) - elapsed
                                if remaining < 60:
                                    eta_str = f"{remaining:.0f}s remaining"
                                else:
                                    eta_str = f"{remaining/60:.1f} min remaining"
                                self.lbl_eta.config(text=eta_str)

                    if pct >= 100:
                        self.lbl_eta.config(text="")
                        self.lbl_pb_pct.config(text="100%")
                        self.btn_upload.state(['!disabled'])
                    elif pct == 0:
                        self.lbl_eta.config(text="")
                        self.btn_upload.state(['!disabled'])

                elif tag == "STATUS":
                    # "Starting upload..." → update status bar but skip the noisy
                    # mid-upload lines ("Finalizing...", "Upload Complete") from the log
                    self.lbl_status.config(text=msg)
                    if msg not in ("Finalizing (Writing Padding)...", "Upload Complete"):
                        self.log("SYS", msg)

                elif tag == "RX":
                    if "high =" in msg:
                        self.vars_rtty["Mark (High)"].set(msg.split('=')[1].strip().split(' ')[0])
                    elif "low =" in msg:
                        self.vars_rtty["Space (Low)"].set(msg.split('=')[1].strip().split(' ')[0])
                    elif "baud =" in msg:
                        self.vars_rtty["Baud"].set(msg.split('=')[1].strip().split(' ')[0])
                    # Suppress the echo of AT+upload and the bare OK that ends the session
                    # (the worker already logged TX; the final OK is implicit in completion)
                    if msg not in ("AT+upload",):
                        self.log("RX", msg)
                else:
                    self.log(tag, msg)
        except queue.Empty:
            pass
        self.root.after(50, self.process_queue)

    def on_close(self):
        self.worker.disconnect()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RttyControllerApp(root)
    root.mainloop()