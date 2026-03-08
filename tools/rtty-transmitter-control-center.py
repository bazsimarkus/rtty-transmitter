# =============================================================================
# rtty-transmitter-control-center.py
#
# Host-side control application for the RTTY/AM Transmitter Card.
# Communicates with an ATmega328P-based board over USB-UART (9600 8N1) using
# a text AT command interface.
#
# Features:
#   - Serial port connection management with auto-refresh
#   - RTTY message transmission with configurable repetitions
#   - Text file transmission with Baudot character filtering and chunking
#   - EEPROM audio upload with hex-dump auto-detection and progress tracking
#   - Direct GPIO control for pins PD2-PD7
#   - Live command log with color-coded TX / RX / SYS / ERR entries
#   - Manual AT command entry
#
# The board firmware accepts commands terminated with CR (\r).
# Replies are terminated with CRLF (\r\n) and end with "OK" or "Error: ...".
#
# Baudot character set note:
#   Only ITA-2 characters are transmittable via AT+send. The firmware silently
#   skips unsupported characters, but this application pre-filters the text so
#   the user can see exactly what will be transmitted.
#
# AT+send payload length:
#   The AVR command buffer is 128 bytes. The command wrapper AT+send=""\r
#   consumes 12 bytes, leaving a maximum of 116 characters per message.
#   Text files are split into chunks of this size before transmission.
#
# Author: Balazs Markus
# =============================================================================

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
# CONSTANTS
# =============================================================================

BAUD_RATE    = 9600      # UART baud rate matching the AVR firmware
EEPROM_SIZE  = 131072    # 25LC1024 capacity in bytes (128 KB)
CMD_TIMEOUT  = 2.0       # Default ACK timeout for queued commands (seconds)

# EEPROM write timing note:
# Each byte write takes up to 5 ms on the 25LC1024. The AVR single-byte UART
# receive buffer is relied upon to catch the next incoming byte while the write
# cycle completes. 4.0 ms is sufficient because serial TX of 2 chars at
# 9600 baud takes ~2.1 ms, keeping the buffer from overflowing.
WRITE_DELAY  = 0.0040    # Inter-byte delay for EEPROM upload (seconds)


# =============================================================================
# SERIAL WORKER
# =============================================================================

class SerialWorker:
    """
    Manages all serial communication with the AVR board in background threads.

    Two persistent daemon threads are started on connect:
      - _writer_thread: drains self.write_queue and sends commands one at a
        time, waiting for an ACK event before proceeding to the next command.
      - _reader_thread: continuously reads incoming bytes, assembles complete
        lines, signals synchronization events, and forwards lines to the UI
        queue.

    A third code path, send_and_wait(), allows transmission tasks (repetitions,
    file chunks) to bypass the write queue and send a single command directly,
    blocking until the board acknowledges or a calculated timeout expires. This
    ensures the board is never flooded with commands it has not yet processed.
    """

    def __init__(self, ui_queue):
        """
        Initialize the worker with a reference to the UI message queue.

        Args:
            ui_queue: A queue.Queue instance used to pass (tag, message) tuples
                      back to the main thread for display in the command log.
        """
        self.ser              = None
        self.ui_queue         = ui_queue
        self.running          = False
        self.write_queue      = queue.Queue()
        self.upload_active    = False

        # Synchronization events used by reader and writer/upload threads.
        self.evt_ack          = threading.Event()   # Fired on "OK" or "Error"
        self.evt_ready        = threading.Event()   # Fired on "READY" (upload)
        self.evt_chunk_ack    = threading.Event()   # Fired on ">" (upload chunk)

        # Lock to serialize direct send_and_wait() calls from task threads.
        self._direct_send_lock = threading.Lock()

    def connect(self, port):
        """
        Open the serial port and start the reader and writer threads.

        Opening the port toggles DTR, which triggers an AVR hardware reset via
        the capacitor on the reset line. A brief delay lets the bootloader
        finish before the input buffer is flushed, ensuring the first received
        line is clean.

        Args:
            port: Device string, e.g. "COM3" or "/dev/ttyUSB0".

        Returns:
            A (success, message) tuple where success is a bool and message is
            a human-readable status or error string.
        """
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            self.running = True
            threading.Thread(target=self._reader_thread, daemon=True).start()
            threading.Thread(target=self._writer_thread, daemon=True).start()
            return True, "Connected"
        except serial.SerialException as e:
            return False, str(e)

    def disconnect(self):
        """
        Signal all threads to stop and close the serial port.

        Sets self.running to False so both daemon threads exit their loops on
        the next iteration. The upload_active flag is also cleared so any
        in-progress upload is aborted gracefully.
        """
        self.running       = False
        self.upload_active = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def send(self, cmd):
        """
        Enqueue a command for asynchronous transmission via the writer thread.

        The command is appended to write_queue and will be sent after all
        previously queued commands have been acknowledged. Suitable for
        single one-off commands. For sequenced multi-command flows (repetitions,
        file chunks) use send_and_wait() instead.

        Args:
            cmd: AT command string without a trailing carriage return.
        """
        if self.running:
            self.write_queue.put(cmd)

    def send_and_wait(self, cmd, timeout):
        """
        Send a command directly to the serial port and block until acknowledged.

        This method bypasses the write_queue so it can be used from task
        threads (repetition loop, file send loop) that must not proceed to the
        next command until the board confirms the current one is complete.

        The _direct_send_lock ensures that at most one send_and_wait() call is
        in flight at any time, preventing concurrent direct writes.

        Args:
            cmd:     AT command string without a trailing carriage return.
            timeout: Maximum number of seconds to wait for an "OK" response.

        Returns:
            A (success, error) tuple. On success: (True, None).
            On failure: (False, human-readable reason string).
        """
        if not self.running or not self.ser or not self.ser.is_open:
            return False, "Not connected"

        with self._direct_send_lock:
            self.evt_ack.clear()
            full_cmd = cmd.strip() + '\r'
            try:
                self.ser.write(full_cmd.encode('ascii'))
                self.ui_queue.put(("TX", cmd.strip()))
            except Exception as e:
                return False, f"Write error: {e}"

            if not self.evt_ack.wait(timeout):
                return False, f"Timeout ({timeout:.1f}s) - no OK from board"
            return True, None

    # -------------------------------------------------------------------------
    # Internal threads
    # -------------------------------------------------------------------------

    def _writer_thread(self):
        """
        Drain the write_queue and send commands one at a time.

        After each command is written to the serial port the thread waits for
        evt_ack to be signaled by the reader thread (or for CMD_TIMEOUT to
        expire). Commands are silently dropped while an EEPROM upload is active
        because the upload protocol uses the serial port directly.
        """
        while self.running:
            try:
                cmd = self.write_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self.upload_active:
                continue

            if self.ser and self.ser.is_open:
                self.evt_ack.clear()
                full_cmd = cmd.strip() + '\r'
                try:
                    self.ser.write(full_cmd.encode('ascii'))
                    self.ui_queue.put(("TX", cmd.strip()))
                except Exception as e:
                    self.ui_queue.put(("ERR", f"Write: {e}"))
                    continue

                if self.evt_ack.wait(CMD_TIMEOUT):
                    time.sleep(0.05)

    def _reader_thread(self):
        """
        Continuously read incoming bytes and dispatch complete lines.

        Bytes are accumulated in a string buffer. Each time a newline is found
        the completed line is cleaned (CR/LF stripped, non-printable characters
        removed) and forwarded to the UI queue as an ("RX", line) tuple.

        Synchronization events are signaled as appropriate:
          - evt_chunk_ack: on receipt of ">" during an active upload session.
          - evt_ready:     on receipt of "READY" during an active upload session.
          - evt_ack:       on receipt of "OK" (always) or any "Error" line when
                           not in upload mode.
        """
        buffer = ""
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    raw_data      = self.ser.read(self.ser.in_waiting)
                    decoded_chunk = raw_data.decode('ascii', errors='ignore')

                    # The upload protocol uses ">" as a per-chunk acknowledgement.
                    # Strip it from the stream before line assembly to avoid
                    # corrupting the text buffer.
                    if self.upload_active and '>' in decoded_chunk:
                        self.evt_chunk_ack.set()
                        decoded_chunk = decoded_chunk.replace('>', '')

                    buffer += decoded_chunk

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        # Strip line endings but preserve internal spaces so
                        # indented banner lines are displayed correctly.
                        # Also drop any null bytes that may appear after reset.
                        line = line.strip('\r\n')
                        line = ''.join(
                            c for c in line if c == ' ' or c.isprintable()
                        )
                        if not line:
                            continue

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

    # -------------------------------------------------------------------------
    # EEPROM upload
    # -------------------------------------------------------------------------

    def upload_eeprom(self, raw_data):
        """
        Start an EEPROM upload in a new daemon thread.

        Args:
            raw_data: Bytes object containing the binary data to upload.
                      Maximum length is EEPROM_SIZE (131072 bytes).
        """
        threading.Thread(
            target=self._upload_task, args=(raw_data,), daemon=True
        ).start()

    def _upload_task(self, data):
        """
        Execute the EEPROM upload protocol.

        The firmware enters upload mode after receiving "AT+upload\\r" and
        responds with "READY\\r\\n". Raw data is then sent as pairs of ASCII
        hex digits in 32-byte chunks. After each chunk the board responds with
        ">" to request the next one. The session ends automatically after
        ~1.5 s of inactivity, after which the firmware writes any remaining
        buffered data and responds with "OK".

        Progress is reported to the UI queue as ("PROGRESS", percentage) tuples
        at the end of each chunk. The progress bar is reset to 0 two seconds
        after completion so it does not appear stuck.

        Args:
            data: Bytes object to upload. Will be padded to a 32-byte boundary
                  with 0xFF fill bytes.
        """
        if len(data) > EEPROM_SIZE:
            self.ui_queue.put(("ERR", f"Data too large: {len(data)} > {EEPROM_SIZE}"))
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
            self.ui_queue.put(("SYS", "Board ready. Sending chunks..."))

            pad_len = CHUNK_SIZE - (len(data) % CHUNK_SIZE)
            if pad_len != CHUNK_SIZE:
                data += b'\xFF' * pad_len

            hex_data     = data.hex().upper()
            total_chunks = len(hex_data) // (CHUNK_SIZE * 2)
            chunks_done  = 0

            for i in range(0, len(hex_data), CHUNK_SIZE * 2):
                if not self.running:
                    break

                chunk_hex = hex_data[i : i + (CHUNK_SIZE * 2)]
                self.evt_chunk_ack.clear()
                self.ser.write(chunk_hex.encode('ascii'))
                self.ser.flush()

                if not self.evt_chunk_ack.wait(5.0):
                    raise Exception(
                        f"Timeout waiting for '>' at offset {i // 2}"
                    )

                chunks_done += 1
                percent = (chunks_done / total_chunks) * 100
                self.ui_queue.put(("PROGRESS", percent))

            # The firmware finalizes the write after an idle timeout of ~1.5 s
            # followed by padding page writes (~80 ms). Allow up to 30 s total.
            if not self.evt_ack.wait(30.0):
                self.ui_queue.put(("WARN", "Final OK timed out"))

            self.ui_queue.put(("PROGRESS", 100))
            self.ui_queue.put(("STATUS", "Upload Complete"))

        except Exception as e:
            self.ui_queue.put(("ERR", f"Upload failed: {e}"))
            try:
                self.ser.write(b'\x1B')
            except Exception:
                pass
        finally:
            self.upload_active = False
            time.sleep(2.0)
            self.ui_queue.put(("PROGRESS", 0))


# =============================================================================
# MAIN GUI
# =============================================================================

class RttyControllerApp:
    """
    Main application window for the RTTY/AM Transmitter Control Center.

    Builds and manages the Tkinter UI, which consists of a left panel with
    tabbed controls and a right panel with a scrollable command log. All serial
    communication is delegated to a SerialWorker instance that runs in daemon
    threads. The main thread polls a shared queue every 50 ms to update the UI
    with incoming messages without blocking.
    """

    # ITA-2 / Baudot characters supported by the firmware (both LTRS and FIGS
    # shift tables). The firmware silently skips unsupported characters, but
    # pre-filtering here keeps the TX log accurate.
    BAUDOT_VALID = set(
        'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
        '0123456789'
        ' -?:$!&#().,\';/"'
    )

    # Maximum payload length for a single AT+send command.
    # AVR CMD_BUF_LEN = 128 bytes. Wrapper AT+send=""\r uses 12 bytes,
    # leaving 116 bytes for the message payload.
    TX_MAX_LEN = 116

    def __init__(self, root):
        """
        Initialize the application, build the UI, and start the queue poller.

        Args:
            root: The Tk root window instance.
        """
        self.root  = root
        self.root.title("RTTY/AM Transmitter Control Center")
        self.root.geometry("1100x600")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.queue                = queue.Queue()
        self.worker               = SerialWorker(self.queue)
        self.connected            = False
        self.pcm_data             = None
        self._upload_start_time   = None
        self._upload_total_bytes  = 0
        self._stop_transmission   = threading.Event()
        self._transmit_file_path  = None

        style = ttk.Style()
        style.theme_use('clam')
        self.create_ui()
        self.process_queue()

    # -------------------------------------------------------------------------
    # UI construction
    # -------------------------------------------------------------------------

    def create_ui(self):
        """
        Build the top-level two-column layout.

        Column 0 (left panel, fixed width 480 px): tabbed controls.
        Column 1 (right panel, expanding): command log.
        """
        self.root.columnconfigure(0, weight=0, minsize=480)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)

        # --- Left panel ---
        left_panel = ttk.Frame(self.root, padding=10)
        left_panel.grid(row=0, column=0, sticky="nsew")

        conn_frame = ttk.Frame(left_panel)
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(conn_frame, text="Port:").pack(side=tk.LEFT)
        self.cb_ports = ttk.Combobox(conn_frame, width=12)
        self.cb_ports.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()
        ttk.Button(
            conn_frame, text="Refresh", command=self.refresh_ports
        ).pack(side=tk.LEFT)
        self.btn_connect = ttk.Button(
            conn_frame, text="Connect", command=self.toggle_connection
        )
        self.btn_connect.pack(side=tk.LEFT, padx=10)
        # Fixed-width label prevents the layout from shifting on text change.
        self.lbl_status = ttk.Label(
            conn_frame, text="Disconnected", foreground="red", width=25
        )
        self.lbl_status.pack(side=tk.LEFT, padx=5)

        self.notebook = ttk.Notebook(left_panel)
        self.notebook.pack(expand=True, fill=tk.BOTH)

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

        # --- Right panel (command log) ---
        right_panel = ttk.Frame(self.root, padding=10)
        right_panel.grid(row=0, column=1, sticky="nsew")

        log_toolbar = ttk.Frame(right_panel)
        log_toolbar.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(
            log_toolbar, text="Command Log", font=("Arial", 10, "bold")
        ).pack(side=tk.LEFT)
        ttk.Button(
            log_toolbar, text="Clear Log", command=self.clear_log
        ).pack(side=tk.RIGHT, padx=5)
        ttk.Button(
            log_toolbar, text="Save Log", command=self.save_log
        ).pack(side=tk.RIGHT)

        self.log_text = scrolledtext.ScrolledText(
            right_panel, state='disabled', font=("Consolas", 9)
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.tag_config("TX",   foreground="blue")
        self.log_text.tag_config("RX",   foreground="green")
        self.log_text.tag_config("ERR",  foreground="red")
        self.log_text.tag_config("SYS",  foreground="gray")
        self.log_text.tag_config("WARN", foreground="orange")

        entry_frame = ttk.Frame(right_panel, padding=(0, 5, 0, 0))
        entry_frame.pack(fill=tk.X)
        ttk.Label(entry_frame, text="Manual CMD:").pack(side=tk.LEFT)
        self.ent_manual = ttk.Entry(entry_frame)
        self.ent_manual.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.ent_manual.bind("<Return>", lambda e: self.send_manual())
        ttk.Button(
            entry_frame, text="Send", command=self.send_manual
        ).pack(side=tk.LEFT)

    def build_rtty_tab(self):
        """Build the RTTY Transmission tab with message, file send, and parameter sections."""
        pad = {'padx': 10, 'pady': 6}

        # --- Transmit Message ---
        frame_msg = ttk.LabelFrame(self.tab_rtty, text="Transmit Message", padding=10)
        frame_msg.pack(fill=tk.X, **pad)

        self.txt_rtty_msg = tk.Text(frame_msg, height=4, width=50)
        self.txt_rtty_msg.pack(fill=tk.X, pady=5)
        self.txt_rtty_msg.insert(
            "1.0", "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG 1234567890"
        )

        rep_row = ttk.Frame(frame_msg)
        rep_row.pack(fill=tk.X, pady=(2, 5))
        ttk.Label(rep_row, text="Repetitions:").pack(side=tk.LEFT)
        self.var_repetitions = tk.IntVar(value=1)
        vcmd = (self.root.register(self._validate_repetitions), '%P')
        self.spn_repetitions = ttk.Spinbox(
            rep_row, from_=1, to=1000, textvariable=self.var_repetitions,
            width=6, validate='key', validatecommand=vcmd
        )
        self.spn_repetitions.pack(side=tk.LEFT, padx=6)

        btn_row = ttk.Frame(frame_msg)
        btn_row.pack(fill=tk.X)
        self.btn_stop_tx = ttk.Button(
            btn_row, text="Stop Transmission",
            command=self.stop_transmission, state='disabled'
        )
        self.btn_stop_tx.pack(side=tk.LEFT)
        ttk.Button(
            btn_row, text="TRANSMIT MESSAGE", command=self.send_rtty_msg
        ).pack(side=tk.RIGHT)

        # --- Send Text File ---
        frame_file = ttk.LabelFrame(self.tab_rtty, text="Send Text File", padding=10)
        frame_file.pack(fill=tk.X, **pad)
        frame_file.columnconfigure(1, weight=1)

        ttk.Label(frame_file, text="File:").grid(row=0, column=0, sticky='w')
        self.lbl_tx_file = ttk.Label(frame_file, text="None", foreground="gray")
        self.lbl_tx_file.grid(row=0, column=1, sticky='ew', padx=5)
        ttk.Button(
            frame_file, text="Browse...", command=self.browse_tx_file
        ).grid(row=0, column=2, sticky='e')

        self.lbl_tx_file_info = ttk.Label(frame_file, text="", foreground="gray")
        self.lbl_tx_file_info.grid(
            row=1, column=0, columnspan=3, sticky='w', pady=(4, 0)
        )

        btn_row2 = ttk.Frame(frame_file)
        btn_row2.grid(row=2, column=0, columnspan=3, sticky='ew', pady=(8, 0))
        self.btn_send_file = ttk.Button(
            btn_row2, text="SEND FILE", command=self.send_tx_file, state='disabled'
        )
        self.btn_send_file.pack(side=tk.RIGHT)

        # --- Current Parameters ---
        frame_set = ttk.LabelFrame(
            self.tab_rtty, text="Current Parameters", padding=10
        )
        frame_set.pack(fill=tk.X, **pad)
        cols = ["Mark (High)", "Space (Low)", "Baud"]
        self.vars_rtty = {}
        for i, col in enumerate(cols):
            ttk.Label(frame_set, text=col).grid(row=0, column=i * 2, sticky='w')
            var = tk.StringVar()
            entry = ttk.Entry(frame_set, textvariable=var, width=10)
            entry.grid(row=0, column=i * 2 + 1, padx=5, sticky='w')
            self.vars_rtty[col] = var
        btn_frame = ttk.Frame(frame_set)
        btn_frame.grid(row=1, column=0, columnspan=6, pady=10)
        ttk.Button(
            btn_frame, text="Read from Board", command=self.get_rtty_params
        ).pack(side=tk.LEFT, padx=5)
        ttk.Button(
            btn_frame, text="Apply to Board", command=self.set_rtty_params
        ).pack(side=tk.LEFT, padx=5)

    def build_audio_tab(self):
        """Build the Audio / EEPROM tab with playback and upload controls."""
        pad = {'padx': 10, 'pady': 10}

        frame_play = ttk.LabelFrame(
            self.tab_audio, text="On-Board Playback", padding=10
        )
        frame_play.pack(fill=tk.X, **pad)
        ttk.Label(frame_play, text="Plays audio stored in EEPROM.").pack(anchor='w')
        ttk.Button(
            frame_play, text="PLAY AUDIO (AT+play)",
            command=lambda: self.worker.send("AT+play")
        ).pack(pady=5)

        frame_up = ttk.LabelFrame(
            self.tab_audio, text="Upload New Audio", padding=10
        )
        frame_up.pack(fill=tk.BOTH, expand=True, **pad)
        frame_up.columnconfigure(0, weight=0)
        frame_up.columnconfigure(1, weight=1)
        frame_up.columnconfigure(2, weight=0)

        ttk.Label(frame_up, text="Selected File:").grid(row=0, column=0, sticky='w')
        self.lbl_file = ttk.Label(frame_up, text="None", foreground="gray")
        self.lbl_file.grid(row=0, column=1, sticky='ew', padx=5)
        ttk.Button(
            frame_up, text="Browse...", command=self.browse_audio
        ).grid(row=0, column=2, sticky='e')

        self.lbl_file_info = ttk.Label(frame_up, text="")
        self.lbl_file_info.grid(row=1, column=0, columnspan=3, sticky='w', pady=5)

        ttk.Separator(frame_up, orient='horizontal').grid(
            row=2, column=0, columnspan=3, sticky='ew', pady=10
        )

        self.btn_upload = ttk.Button(
            frame_up, text="Upload to EEPROM",
            state='disabled', command=self.start_upload
        )
        self.btn_upload.grid(row=3, column=0, columnspan=3, sticky='ew')

        pb_frame = ttk.Frame(frame_up)
        pb_frame.grid(row=4, column=0, columnspan=3, sticky='ew', pady=(10, 2))
        pb_frame.columnconfigure(0, weight=1)
        self.pb_upload = ttk.Progressbar(
            pb_frame, orient='horizontal', mode='determinate'
        )
        self.pb_upload.grid(row=0, column=0, sticky='ew')
        self.lbl_pb_pct = ttk.Label(pb_frame, text="0%", width=5, anchor='e')
        self.lbl_pb_pct.grid(row=0, column=1, padx=(6, 0))

        self.lbl_eta = ttk.Label(frame_up, text="", foreground="gray")
        self.lbl_eta.grid(row=5, column=0, columnspan=3, sticky='w')

    def build_gpio_tab(self):
        """Build the GPIO Control tab with HIGH/LOW buttons for pins PD2-PD7."""
        frame = ttk.Frame(self.tab_gpio, padding=20)
        frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(
            frame, text="Direct GPIO Control (PD2 - PD7)",
            font=("Arial", 10, "bold")
        ).pack(pady=(0, 20))
        grid_frame = ttk.Frame(frame)
        grid_frame.pack()
        for pin in range(6):
            p_frame = ttk.Frame(
                grid_frame, borderwidth=1, relief="sunken", padding=10
            )
            p_frame.grid(row=pin // 3, column=pin % 3, padx=10, pady=10)
            ttk.Label(
                p_frame, text=f"GPIO {pin}\n(PD{pin + 2})", justify='center'
            ).pack()
            ttk.Button(
                p_frame, text="HIGH",
                command=lambda p=pin: self.worker.send(f"AT+write={p},1")
            ).pack(pady=2)
            ttk.Button(
                p_frame, text="LOW",
                command=lambda p=pin: self.worker.send(f"AT+write={p},0")
            ).pack(pady=2)

    def build_sys_tab(self):
        """Build the System tab with connection test, factory reset, and help commands."""
        frame = ttk.Frame(self.tab_sys, padding=20)
        frame.pack()
        ttk.Button(
            frame, text="Check Connection (AT)",
            command=lambda: self.worker.send("AT")
        ).pack(fill=tk.X, pady=5)
        ttk.Button(
            frame, text="Factory Reset (AT+reset)",
            command=lambda: self.worker.send("AT+reset")
        ).pack(fill=tk.X, pady=5)
        ttk.Button(
            frame, text="Help (AT+help)",
            command=lambda: self.worker.send("AT+help")
        ).pack(fill=tk.X, pady=5)

    # -------------------------------------------------------------------------
    # Connection management
    # -------------------------------------------------------------------------

    def refresh_ports(self):
        """Scan for available serial ports and populate the port combobox."""
        ports = serial.tools.list_ports.comports()
        self.cb_ports['values'] = [p.device for p in ports]
        if ports:
            self.cb_ports.current(0)

    def toggle_connection(self):
        """
        Connect to or disconnect from the currently selected serial port.

        On successful connection, an "AT" command is sent to verify the link
        and the current RTTY parameters are read back from the board.
        """
        if not self.connected:
            port = self.cb_ports.get()
            if not port:
                return
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

    # -------------------------------------------------------------------------
    # Audio / EEPROM upload
    # -------------------------------------------------------------------------

    def browse_audio(self):
        """
        Open a file dialog for audio/binary file selection and validate the file.

        Supports three input types detected automatically:
          - WAV file (RIFF header): prompts the user before uploading as raw data.
          - Hex-dump text file: converted to binary before upload.
          - Raw binary: uploaded as-is.

        Updates the file info label with size and estimated upload time. Enables
        the Upload button only if the file fits within the 128 KB EEPROM limit.
        """
        filename = filedialog.askopenfilename(
            filetypes=[
                ("All Files", "*.*"),
                ("Text Files", "*.txt"),
                ("WAV Files", "*.wav"),
            ]
        )
        if not filename:
            return

        self.lbl_file.config(text=os.path.basename(filename), foreground="black")
        try:
            self.log("SYS", f"Loading {os.path.basename(filename)}...")
            with open(filename, "rb") as f:
                raw_content = f.read()

            is_wav       = raw_content.startswith(b'RIFF')
            is_hex_text  = False
            text_content = ""

            try:
                text_content = raw_content.decode('ascii').strip()
                if all(
                    c in '0123456789ABCDEFabcdef \r\n' for c in text_content
                ):
                    is_hex_text = True
            except Exception:
                pass

            final_data = raw_content

            if is_wav:
                self.log("WARN", "WAV header detected. Will be uploaded as raw binary.")
                msg = (
                    f"WAV Header detected.\n\n"
                    f"Type: WAV File\nSize: {len(raw_content)} bytes\n\n"
                    f"Upload as-is?"
                )
                if not messagebox.askyesno("WAV Detected", msg):
                    self.pcm_data = None
                    self.btn_upload.state(['disabled'])
                    return

            elif is_hex_text:
                self.log("SYS", "Hex-dump text detected. Converting to binary...")
                clean_hex = (
                    text_content
                    .replace(' ', '')
                    .replace('\r', '')
                    .replace('\n', '')
                )
                try:
                    final_data = binascii.unhexlify(clean_hex)
                    self.log("SYS", f"Converted to {len(final_data)} binary bytes.")
                except Exception as e:
                    self.log("ERR", f"Conversion failed: {e}")
                    messagebox.showerror("Error", "Could not convert hex text to binary.")
                    return

            size = len(final_data)

            # Estimate upload time.
            # Each raw byte becomes 2 ASCII hex digits. At 9600 baud with 10
            # bits per character, TX time per byte is 2 * 10 / 9600 seconds.
            # EEPROM page writes add ~5 ms per 32-byte page. The firmware also
            # applies a 1.5 s idle timeout before finalizing, plus ~80 ms for
            # 16 padding pages.
            num_pages = (size + 31) // 32
            tx_ms     = size * (2 * 10 / 9600) * 1000
            write_ms  = num_pages * 5
            idle_ms   = 1500
            pad_ms    = 16 * 5
            total_sec = (tx_ms + write_ms + idle_ms + pad_ms) / 1000.0

            if total_sec < 60:
                eta_str = f"~{total_sec:.0f} s"
            else:
                eta_str = f"~{total_sec / 60:.1f} min"

            info = f"{size:,} bytes  -  Est. upload time: {eta_str}"
            self.lbl_file_info.config(text=info)

            if size > EEPROM_SIZE:
                self.lbl_file_info.config(
                    text=info + "  ERROR: exceeds 128 KB!", foreground="red"
                )
                self.btn_upload.state(['disabled'])
                self.pcm_data = None
            else:
                self.lbl_file_info.config(foreground="black")
                self.btn_upload.state(['!disabled'])
                self.pcm_data = final_data
                preview = final_data[:8].hex().upper()
                self.log("SYS", f"First 8 bytes: {preview}")

        except Exception as e:
            messagebox.showerror("File Error", str(e))

    def start_upload(self):
        """
        Begin the EEPROM upload for the currently loaded audio file.

        Resets the progress bar and ETA label, disables the Upload button to
        prevent double-submission, and delegates to SerialWorker.upload_eeprom().
        """
        if not self.connected:
            return
        if self.pcm_data:
            self._upload_start_time  = None
            self._upload_total_bytes = len(self.pcm_data)
            self.lbl_eta.config(text="")
            self.lbl_pb_pct.config(text="0%")
            self.btn_upload.state(['disabled'])
            self.worker.upload_eeprom(self.pcm_data)

    # -------------------------------------------------------------------------
    # RTTY transmission
    # -------------------------------------------------------------------------

    def _validate_repetitions(self, new_val):
        """
        Validate the repetitions spinbox entry.

        Allows only integers in the range 1-1000. An empty string is permitted
        to allow the user to clear and retype the field.

        Args:
            new_val: The proposed new value as a string (from Tkinter validate).

        Returns:
            True if the value is acceptable, False otherwise.
        """
        if new_val == '':
            return True
        try:
            v = int(new_val)
            return 1 <= v <= 1000
        except ValueError:
            return False

    def _filter_baudot(self, text):
        """
        Upper-case text and remove characters that cannot be transmitted via ITA-2.

        Newline and carriage return characters are replaced with a space rather
        than being preserved, because embedded line breaks in an AT+send payload
        cause the board to echo partial lines interleaved with the command,
        which produces garbled output in the command log. Text files are
        therefore transmitted as continuous single-line chunks.

        The firmware's baudot_code() function silently skips unsupported
        characters. Pre-filtering here ensures the chunk count and TX log
        accurately reflect what will actually be transmitted.

        Args:
            text: Input string, may contain any Unicode characters.

        Returns:
            A string containing only upper-case characters present in
            BAUDOT_VALID with all line break characters replaced by spaces.
        """
        upper = text.upper()
        upper = upper.replace('\r', ' ').replace('\n', ' ')
        return ''.join(c for c in upper if c in self.BAUDOT_VALID)

    def _calc_tx_timeout(self, msg_len):
        """
        Calculate a transmission timeout for a single AT+send command.

        The timeout is based on the current baud rate setting and the message
        length. Each ITA-2 character occupies 7.5 bit periods (1 start + 5 data
        + 1.5 stop). Worst-case assumes every character requires a shift
        character, doubling the character count. A 2x safety margin plus 3 s
        headroom for UART round-trip and board processing is applied on top.

        Falls back to 30 s when the Baud field is empty or contains an invalid
        value, so a timeout always fires rather than hanging indefinitely.

        Args:
            msg_len: Number of characters in the message payload.

        Returns:
            Timeout duration in seconds as a float.
        """
        MIN_TIMEOUT = 8.0
        FALLBACK    = 30.0
        try:
            baud = float(self.vars_rtty["Baud"].get())
            if baud <= 0:
                raise ValueError
        except (ValueError, AttributeError, tk.TclError):
            return FALLBACK

        bits_per_char = 7.5
        # Preamble: CR + LF + shift = 3 characters. Worst case: each payload
        # character is preceded by a shift character (2x multiplier).
        worst_chars = 3 + msg_len * 2
        tx_seconds  = (worst_chars * bits_per_char) / baud
        timeout     = max(MIN_TIMEOUT, tx_seconds * 2.0 + 3.0)
        return timeout

    def stop_transmission(self):
        """
        Request cancellation of the active repetition or file send task.

        Sets the _stop_transmission event, which is checked by the task thread
        between each command. The current in-flight transmission will complete
        before the task exits.
        """
        self._stop_transmission.set()
        self.log("SYS", "Stop requested. Transmission will halt after the current send.")

    def send_rtty_msg(self):
        """
        Read the message text box and start a repetition task in a daemon thread.

        Strips double-quote characters (which are used as delimiters in the
        AT+send command syntax and would break the firmware parser). Reads the
        repetitions spinbox and clamps the value to 1-1000.
        """
        msg = self.txt_rtty_msg.get("1.0", tk.END).strip()
        if not msg:
            return
        msg = msg.replace('"', '')
        try:
            reps = int(self.var_repetitions.get())
        except (ValueError, tk.TclError):
            reps = 1
        reps = max(1, min(1000, reps))
        self._stop_transmission.clear()
        self.btn_stop_tx.state(['!disabled'])
        threading.Thread(
            target=self._repeat_send_task, args=(msg, reps), daemon=True
        ).start()

    def _repeat_send_task(self, msg, reps):
        """
        Send the same message a given number of times, waiting for OK between each.

        Each call to send_and_wait() blocks until the board responds with OK or
        the calculated timeout expires. If the board does not acknowledge or the
        stop event is set, the loop exits immediately.

        Args:
            msg:  Message string (double-quotes already removed).
            reps: Number of times to transmit the message (1-1000).
        """
        timeout = self._calc_tx_timeout(len(msg))
        self.queue.put((
            "SYS",
            f"Starting {reps}x repetition(s), timeout per TX: {timeout:.1f}s"
        ))
        for i in range(reps):
            if self._stop_transmission.is_set():
                self.queue.put(("SYS", f"Stopped after {i}/{reps} repetition(s)."))
                break
            ok, err = self.worker.send_and_wait(f'AT+send="{msg}"', timeout)
            if not ok:
                self.queue.put(("ERR", f"Rep {i + 1}/{reps} failed: {err}"))
                break
        else:
            self.queue.put(("SYS", f"All {reps} repetition(s) complete."))
        self.root.after(0, lambda: self.btn_stop_tx.state(['disabled']))

    # -------------------------------------------------------------------------
    # Text file transmission
    # -------------------------------------------------------------------------

    def browse_tx_file(self):
        """
        Open a file dialog to select a text file for RTTY transmission.

        The file is read, converted to upper-case, filtered to Baudot-valid
        characters (with newlines replaced by spaces), and split into chunks of
        TX_MAX_LEN characters. The info label is updated with the character
        counts and chunk count. The Send File button is enabled only if at least
        one transmittable character exists in the file.
        """
        filename = filedialog.askopenfilename(
            filetypes=[("Text Files", "*.txt"), ("All Files", "*.*")]
        )
        if not filename:
            return
        self._transmit_file_path = filename
        self.lbl_tx_file.config(text=os.path.basename(filename), foreground="black")
        try:
            with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
                raw = f.read()
            filtered = self._filter_baudot(raw)
            chunks = [
                filtered[i : i + self.TX_MAX_LEN]
                for i in range(0, len(filtered), self.TX_MAX_LEN)
            ]
            info = (
                f"{len(raw)} chars in file - "
                f"{len(filtered)} transmittable - "
                f"{len(chunks)} chunk(s) of max {self.TX_MAX_LEN} chars"
            )
            self.lbl_tx_file_info.config(text=info, foreground="gray")
            if chunks:
                self.btn_send_file.state(['!disabled'])
            else:
                self.btn_send_file.state(['disabled'])
                self.lbl_tx_file_info.config(
                    text="No transmittable characters found in file.",
                    foreground="red"
                )
        except Exception as e:
            messagebox.showerror("File Error", str(e))
            self.btn_send_file.state(['disabled'])

    def send_tx_file(self):
        """
        Read the selected file and start the file send task in a daemon thread.

        Re-reads and re-filters the file at send time to ensure the content
        is current. Disables the Send File button and enables the Stop button
        for the duration of the task.
        """
        if not self._transmit_file_path:
            return
        try:
            with open(
                self._transmit_file_path, 'r', encoding='utf-8', errors='ignore'
            ) as f:
                raw = f.read()
        except Exception as e:
            messagebox.showerror("File Error", str(e))
            return
        filtered = self._filter_baudot(raw)
        if not filtered:
            messagebox.showwarning("No Content", "No transmittable characters found.")
            return
        chunks = [
            filtered[i : i + self.TX_MAX_LEN]
            for i in range(0, len(filtered), self.TX_MAX_LEN)
        ]
        self._stop_transmission.clear()
        self.btn_stop_tx.state(['!disabled'])
        self.btn_send_file.state(['disabled'])
        threading.Thread(
            target=self._file_send_task, args=(chunks,), daemon=True
        ).start()

    def _file_send_task(self, chunks):
        """
        Transmit a list of pre-chunked strings sequentially over RTTY.

        Each chunk is sent with send_and_wait(), which blocks until the board
        acknowledges with OK or the timeout expires. If the board fails to
        acknowledge or the stop event is set, the task exits immediately without
        sending further chunks.

        Args:
            chunks: List of strings, each at most TX_MAX_LEN characters, already
                    filtered to Baudot-valid characters with no line breaks.
        """
        total = len(chunks)
        for idx, chunk in enumerate(chunks):
            if self._stop_transmission.is_set():
                self.queue.put((
                    "SYS", f"File send stopped after chunk {idx}/{total}."
                ))
                break
            timeout = self._calc_tx_timeout(len(chunk))
            ok, err = self.worker.send_and_wait(f'AT+send="{chunk}"', timeout)
            if not ok:
                self.queue.put((
                    "ERR",
                    f"Chunk {idx + 1}/{total} failed: {err}. Aborting file send."
                ))
                break
            self.queue.put(("SYS", f"File chunk {idx + 1}/{total} sent OK."))
        else:
            self.queue.put(("SYS", f"File send complete - {total} chunk(s) sent."))
        self.root.after(0, lambda: self.btn_stop_tx.state(['disabled']))
        self.root.after(0, lambda: self.btn_send_file.state(['!disabled']))

    # -------------------------------------------------------------------------
    # RTTY parameter read/write
    # -------------------------------------------------------------------------

    def get_rtty_params(self):
        """Query the board for the current mark frequency, space frequency, and baud rate."""
        self.worker.send("AT+get=high")
        self.worker.send("AT+get=low")
        self.worker.send("AT+get=baud")

    def set_rtty_params(self):
        """
        Apply the values from the parameter fields to the board.

        Sends AT+set commands only for non-empty fields, then schedules a
        read-back after 500 ms to confirm the values were accepted.
        """
        try:
            h = self.vars_rtty["Mark (High)"].get()
            l = self.vars_rtty["Space (Low)"].get()
            b = self.vars_rtty["Baud"].get()
            if h:
                self.worker.send(f"AT+set=high,{h}")
            if l:
                self.worker.send(f"AT+set=low,{l}")
            if b:
                self.worker.send(f"AT+set=baud,{b}")
            self.root.after(500, self.get_rtty_params)
        except Exception:
            pass

    # -------------------------------------------------------------------------
    # Manual command
    # -------------------------------------------------------------------------

    def send_manual(self):
        """Read the manual command entry field and enqueue the command for transmission."""
        cmd = self.ent_manual.get()
        if cmd:
            self.worker.send(cmd)
            self.ent_manual.delete(0, tk.END)

    # -------------------------------------------------------------------------
    # Command log
    # -------------------------------------------------------------------------

    def log(self, tag, msg):
        """
        Append a timestamped entry to the command log.

        The log widget is briefly set to normal state for insertion, then
        returned to disabled to prevent user edits. The view is scrolled to
        the end after each insertion.

        Args:
            tag: Category string used as both the log prefix and the Tkinter
                 text tag for color coding. Valid tags: TX, RX, SYS, ERR, WARN.
            msg: The message text to display.
        """
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"[{timestamp}] [{tag}] {msg}\n", tag)
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

    def clear_log(self):
        """Delete all content from the command log widget."""
        self.log_text.config(state='normal')
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state='disabled')

    def save_log(self):
        """
        Prompt the user for a file path and save the current log contents to disk.

        Writes the full text of the log widget to a plain text file. Shows an
        error dialog if the write fails.
        """
        fn = filedialog.asksaveasfilename(
            defaultextension=".log",
            filetypes=[("Log Files", "*.log")]
        )
        if fn:
            try:
                with open(fn, 'w') as f:
                    f.write(self.log_text.get(1.0, tk.END))
                self.log("SYS", f"Log saved to {fn}")
            except Exception as e:
                messagebox.showerror("Save Error", str(e))

    # -------------------------------------------------------------------------
    # Queue poller
    # -------------------------------------------------------------------------

    def process_queue(self):
        """
        Drain the worker-to-UI message queue and update the UI accordingly.

        Called every 50 ms via root.after(). Processes all pending messages in
        a single pass to avoid latency accumulation under heavy load.

        Message tag handling:
          - PROGRESS: updates the upload progress bar and ETA label.
          - STATUS:   updates the status bar label and logs to SYS (except for
                      mid-upload noise lines that are intentionally suppressed).
          - RX:       parses parameter responses to update the RTTY fields, then
                      logs the line. The AT+upload echo is suppressed because the
                      worker already logged it as TX.
          - All other tags (TX, SYS, ERR, WARN): logged directly.
        """
        try:
            while True:
                tag, msg = self.queue.get_nowait()

                if tag == "PROGRESS":
                    pct = float(msg)
                    self.pb_upload['value'] = pct
                    self.lbl_pb_pct.config(text=f"{pct:.0f}%")

                    if 0 < pct < 100:
                        if self._upload_start_time is None:
                            self._upload_start_time = time.time()
                        else:
                            elapsed = time.time() - self._upload_start_time
                            if elapsed > 0:
                                remaining = elapsed / (pct / 100.0) - elapsed
                                if remaining < 60:
                                    eta_str = f"{remaining:.0f}s remaining"
                                else:
                                    eta_str = f"{remaining / 60:.1f} min remaining"
                                self.lbl_eta.config(text=eta_str)

                    if pct >= 100:
                        self.lbl_eta.config(text="")
                        self.lbl_pb_pct.config(text="100%")
                        self.btn_upload.state(['!disabled'])
                    elif pct == 0:
                        self.lbl_eta.config(text="")
                        self.btn_upload.state(['!disabled'])

                elif tag == "STATUS":
                    if msg not in ("Finalizing (Writing Padding)...", "Upload Complete"):
                        self.log("SYS", msg)

                elif tag == "RX":
                    if "high =" in msg:
                        self.vars_rtty["Mark (High)"].set(
                            msg.split('=')[1].strip().split(' ')[0]
                        )
                    elif "low =" in msg:
                        self.vars_rtty["Space (Low)"].set(
                            msg.split('=')[1].strip().split(' ')[0]
                        )
                    elif "baud =" in msg:
                        self.vars_rtty["Baud"].set(
                            msg.split('=')[1].strip().split(' ')[0]
                        )
                    if msg not in ("AT+upload",):
                        self.log("RX", msg)

                else:
                    self.log(tag, msg)

        except queue.Empty:
            pass
        self.root.after(50, self.process_queue)

    # -------------------------------------------------------------------------
    # Application lifecycle
    # -------------------------------------------------------------------------

    def on_close(self):
        """Disconnect the serial worker and destroy the root window on exit."""
        self.worker.disconnect()
        self.root.destroy()


# =============================================================================
# ENTRY POINT
# =============================================================================

if __name__ == "__main__":
    root = tk.Tk()
    app  = RttyControllerApp(root)
    root.mainloop()