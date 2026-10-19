import json
import math
import queue
import re
import subprocess
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk

try:
    import speech_recognition as sr
except ImportError:
    sr = None

try:
    import pyttsx3
except ImportError:
    pyttsx3 = None

try:
    import sounddevice as sd
    from vosk import KaldiRecognizer, Model
except ImportError:
    sd = None
    KaldiRecognizer = None
    Model = None


APP_DIR = Path(__file__).resolve().parent
MAP_FILE = APP_DIR / "campus_map.json"
VOSK_MODEL_DIRS = [
    APP_DIR / "model",
    APP_DIR / "vosk-model-small-en-us-0.15",
    APP_DIR / "vosk-model-en-us-0.22",
]
YES_WORDS = {"yes", "yeah", "yep", "correct", "right", "confirm", "okay", "ok"}
NO_WORDS = {"no", "nope", "wrong", "incorrect", "again", "not correct"}


@dataclass
class Node:
    id: str
    label: str
    x: float
    y: float
    aliases: list[str]


class CampusMap:
    def __init__(self, path: Path):
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        self.units = payload.get("units", "meters")
        self.nodes = {item["id"]: Node(**item) for item in payload["nodes"]}
        self.edges = payload["edges"]
        self.alias_lookup = {}
        for node in self.nodes.values():
            names = [node.label, node.id.replace("_", " "), *node.aliases]
            for name in names:
                self.alias_lookup[self._clean(name)] = node.id

    def _clean(self, text: str) -> str:
        return re.sub(r"[^a-z0-9 ]+", "", text.lower()).strip()

    def match_place(self, spoken_text: str) -> Node | None:
        cleaned = self._clean(spoken_text)
        if cleaned in self.alias_lookup:
            return self.nodes[self.alias_lookup[cleaned]]
        for alias, node_id in self.alias_lookup.items():
            if alias in cleaned or cleaned in alias:
                return self.nodes[node_id]
        return None

    def neighbors(self, node_id: str) -> list[tuple[str, float]]:
        result = []
        for edge in self.edges:
            if edge["from"] == node_id:
                result.append((edge["to"], edge["distance"]))
            if edge.get("bidirectional") and edge["to"] == node_id:
                result.append((edge["from"], edge["distance"]))
        return result

    def shortest_path(self, start_id: str, end_id: str) -> tuple[list[str], float]:
        distances = {node_id: math.inf for node_id in self.nodes}
        previous = {}
        unvisited = set(self.nodes)
        distances[start_id] = 0

        while unvisited:
            current = min(unvisited, key=lambda item: distances[item])
            unvisited.remove(current)
            if current == end_id:
                break
            for neighbor_id, distance in self.neighbors(current):
                if neighbor_id not in unvisited:
                    continue
                score = distances[current] + distance
                if score < distances[neighbor_id]:
                    distances[neighbor_id] = score
                    previous[neighbor_id] = current

        path = []
        cursor = end_id
        while cursor:
            path.insert(0, cursor)
            cursor = previous.get(cursor)
        if not path or path[0] != start_id:
            return [], math.inf
        return path, distances[end_id]


class VoiceIO:
    def __init__(self, log_callback):
        self.log_callback = log_callback
        self.recognizer = sr.Recognizer() if sr else None
        self.vosk_model = self._load_vosk_model()
        self.speech_lock = threading.Lock()
        self.tts = None
        if pyttsx3:
            try:
                self.tts = pyttsx3.init()
                self.tts.setProperty("rate", 165)
            except Exception as exc:
                self.log_callback(f"pyttsx3 unavailable: {exc}")

    def _load_vosk_model(self):
        if not Model:
            self.log_callback("Vosk is not installed.")
            return None
        for model_dir in VOSK_MODEL_DIRS:
            if model_dir.exists():
                try:
                    self.log_callback(f"Using Vosk model: {model_dir.name}")
                    return Model(str(model_dir))
                except Exception as exc:
                    self.log_callback(f"Could not load Vosk model {model_dir}: {exc}")
        self.log_callback("No Vosk model folder found. Expected ./model or ./vosk-model-small-en-us-0.15")
        return None

    def speak(self, text: str):
        self.log_callback(f"Assistant: {text}")
        with self.speech_lock:
            if self.tts:
                try:
                    self.tts.say(text)
                    self.tts.runAndWait()
                    return
                except Exception as exc:
                    self.log_callback(f"pyttsx3 speech failed: {exc}")
            self._speak_windows(text)

    def speak_navigation(self, text: str):
        self.log_callback(f"Navigation: {text}")
        with self.speech_lock:
            if self._speak_windows(text):
                return
            if self.tts:
                try:
                    self.tts.say(text)
                    self.tts.runAndWait()
                except Exception as exc:
                    self.log_callback(f"Navigation speech failed: {exc}")

    def _speak_windows(self, text: str) -> bool:
        if sys.platform.startswith("win"):
            safe_text = json.dumps(text)
            command = (
                "Add-Type -AssemblyName System.Speech; "
                "$speaker = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                f"$speaker.Speak({safe_text})"
            )
            try:
                subprocess.run(
                    ["powershell", "-NoProfile", "-Command", command],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                return True
            except Exception as exc:
                self.log_callback(f"Speech output failed: {exc}")
        return False

    def listen(self, prompt: str, typed_fallback, choices: list[str] | None = None, attempts: int = 1):
        self.speak(prompt)
        if choices:
            text = self._listen_vosk_choices(choices, attempts)
            if text:
                self.log_callback(f"You said: {text}")
                return text
        if sys.platform.startswith("win") and choices:
            for attempt in range(1, attempts + 1):
                text = self._listen_windows_choices(choices, attempt, attempts)
                if text:
                    self.log_callback(f"You said: {text}")
                    return text
                if attempt < attempts:
                    self.speak("I did not hear that. Please say it again.")
        if not self.recognizer:
            return typed_fallback(
                "I could not hear a matching voice command. Check that microphone access is enabled, "
                "then type the same words here if needed."
            )
        try:
            with sr.Microphone() as source:
                self.log_callback("Listening...")
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                audio = self.recognizer.listen(source, timeout=6, phrase_time_limit=5)
            text = self.recognizer.recognize_google(audio)
            self.log_callback(f"You said: {text}")
            return text
        except Exception as exc:
            return typed_fallback(f"Voice input failed: {exc}")

    def _listen_vosk_choices(self, choices: list[str], attempts: int) -> str | None:
        if not self.vosk_model or not sd or not KaldiRecognizer:
            return None
        unique_choices = sorted({choice.strip().lower() for choice in choices if choice.strip()})
        grammar = json.dumps(unique_choices + ["[unk]"])
        sample_rate = 16000
        for attempt in range(1, attempts + 1):
            audio_queue = queue.Queue()
            recognizer = KaldiRecognizer(self.vosk_model, sample_rate, grammar)

            def callback(indata, frames, time_info, status):
                if status:
                    self.log_callback(f"Vosk microphone status: {status}")
                audio_queue.put(bytes(indata))

            self.log_callback(f"Vosk listening... attempt {attempt} of {attempts}")
            try:
                with sd.RawInputStream(
                    samplerate=sample_rate,
                    blocksize=8000,
                    dtype="int16",
                    channels=1,
                    callback=callback,
                ):
                    deadline = time.time() + 7
                    while time.time() < deadline:
                        try:
                            data = audio_queue.get(timeout=0.8)
                        except queue.Empty:
                            continue
                        if recognizer.AcceptWaveform(data):
                            result = json.loads(recognizer.Result())
                            text = result.get("text", "").strip()
                            if text and text != "[unk]":
                                return text
                    final_result = json.loads(recognizer.FinalResult())
                    text = final_result.get("text", "").strip()
                    if text and text != "[unk]":
                        return text
            except Exception as exc:
                self.log_callback(f"Vosk microphone failed: {exc}")
                return None
            if attempt < attempts:
                self.speak("I did not hear that. Please say it again.")
        return None

    def _listen_windows_choices(self, choices: list[str], attempt: int, total_attempts: int) -> str | None:
        unique_choices = sorted({choice.strip().lower() for choice in choices if choice.strip()})
        if not unique_choices:
            return None
        time.sleep(0.7)
        script = f"""
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Speech
$phrases = @'
{json.dumps(unique_choices)}
'@ | ConvertFrom-Json
$recognizerInfo = [System.Speech.Recognition.SpeechRecognitionEngine]::InstalledRecognizers() |
  Where-Object {{ $_.Enabled -ne $false }} |
  Select-Object -First 1
if ($null -eq $recognizerInfo) {{
  $recognizer = New-Object System.Speech.Recognition.SpeechRecognitionEngine
}} else {{
  $recognizer = New-Object System.Speech.Recognition.SpeechRecognitionEngine($recognizerInfo)
}}
try {{
  $recognizer.InitialSilenceTimeout = [TimeSpan]::FromSeconds(10)
  $recognizer.EndSilenceTimeout = [TimeSpan]::FromSeconds(2)
  $recognizer.BabbleTimeout = [TimeSpan]::FromSeconds(10)
}} catch {{
  # Older recognizer engines may not expose every timeout property.
}}
$speechChoices = New-Object System.Speech.Recognition.Choices
foreach ($phrase in $phrases) {{
  [void]$speechChoices.Add([string]$phrase)
}}
$builder = New-Object System.Speech.Recognition.GrammarBuilder
$builder.Append($speechChoices)
$grammar = New-Object System.Speech.Recognition.Grammar($builder)
$recognizer.LoadGrammar($grammar)
try {{
  $dictation = New-Object System.Speech.Recognition.DictationGrammar
  $recognizer.LoadGrammar($dictation)
}} catch {{
  # Some Windows editions do not expose dictation grammar to desktop apps.
}}
$recognizer.SetInputToDefaultAudioDevice()
Write-Error "VOICE_DIAGNOSTIC: Recognizer=$($recognizer.RecognizerInfo.Description); AudioState=$($recognizer.AudioState); AudioLevel=$($recognizer.AudioLevel)"
$result = $recognizer.Recognize([TimeSpan]::FromSeconds(15))
Write-Error "VOICE_DIAGNOSTIC: AfterRecognize AudioState=$($recognizer.AudioState); AudioLevel=$($recognizer.AudioLevel)"
$recognizer.Dispose()
if ($null -ne $result) {{
  Write-Error "VOICE_DIAGNOSTIC: Confidence=$($result.Confidence)"
  Write-Output $result.Text
  exit 0
}}
Write-Error "VOICE_DIAGNOSTIC: Recognize returned no result."
exit 2
"""
        try:
            self.log_callback(f"Listening... attempt {attempt} of {total_attempts}")
            result = subprocess.run(
                ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", "-"],
                input=script,
                text=True,
                capture_output=True,
                timeout=25,
                check=False,
            )
        except Exception as exc:
            self.log_callback(f"Windows speech recognition failed: {exc}")
            return None
        if result.stderr.strip():
            for line in result.stderr.strip().splitlines():
                if "VOICE_DIAGNOSTIC:" in line or "Exception" in line or "error" in line.lower():
                    self.log_callback(line.strip())
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip().splitlines()[-1]
        if result.returncode == 2:
            self.log_callback("No speech was recognized before the timeout.")
        elif result.returncode != 0:
            self.log_callback(f"Windows speech recognition exited with code {result.returncode}.")
        return None


class NavigatorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Campus Offline Voice Navigator")
        self.geometry("1050x760")
        self.minsize(900, 650)
        self.configure(bg="#eef3f0")

        self.map = CampusMap(MAP_FILE)
        self.events = queue.Queue()
        self.voice = VoiceIO(self.log)
        self.start_node = None
        self.end_node = None
        self.route = []
        self.commands = []
        self.scale = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.audio_index = 0

        self._build_ui()
        self._draw_map()
        self.after(100, self._process_events)

    def _build_ui(self):
        root = ttk.Frame(self, padding=14)
        root.pack(fill=tk.BOTH, expand=True)
        root.columnconfigure(0, weight=3)
        root.columnconfigure(1, weight=2)
        root.rowconfigure(0, weight=1)

        map_frame = ttk.Frame(root)
        map_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        map_frame.rowconfigure(0, weight=1)
        map_frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(map_frame, bg="#dfe9e5", highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.canvas.bind("<Configure>", lambda _event: self._draw_map())

        panel = ttk.Frame(root)
        panel.grid(row=0, column=1, sticky="nsew")
        panel.columnconfigure(0, weight=1)
        panel.rowconfigure(3, weight=1)

        title = ttk.Label(panel, text="Campus Voice Navigator", font=("Segoe UI", 18, "bold"))
        title.grid(row=0, column=0, sticky="w", pady=(0, 6))

        help_text = (
            "Click Start Voice Navigation. The app will ask for start and end "
            "places, confirm each by voice, then show the route and alerts."
        )
        ttk.Label(panel, text=help_text, wraplength=360).grid(row=1, column=0, sticky="w", pady=(0, 12))

        controls = ttk.Frame(panel)
        controls.grid(row=2, column=0, sticky="ew", pady=(0, 12))
        controls.columnconfigure((0, 1), weight=1)

        ttk.Button(
            controls,
            text="Start Voice Navigation",
            command=self.start_voice_flow,
        ).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        ttk.Button(
            controls,
            text="Play Audio Commands",
            command=self.play_audio_commands,
        ).grid(row=0, column=1, sticky="ew", padx=(6, 0))
        ttk.Button(
            controls,
            text="Test Microphone",
            command=self.test_microphone,
        ).grid(row=1, column=0, sticky="ew", padx=(0, 6), pady=(8, 0))
        ttk.Button(
            controls,
            text="Open Mic Settings",
            command=self.open_microphone_settings,
        ).grid(row=1, column=1, sticky="ew", padx=(6, 0), pady=(8, 0))

        self.status = tk.StringVar(value="Ready. Waiting for start.")
        ttk.Label(panel, textvariable=self.status, font=("Segoe UI", 11, "bold"), wraplength=360).grid(
            row=3, column=0, sticky="new", pady=(0, 8)
        )

        list_frame = ttk.LabelFrame(panel, text="Navigation Commands", padding=10)
        list_frame.grid(row=4, column=0, sticky="nsew", pady=(0, 12))
        list_frame.rowconfigure(0, weight=1)
        list_frame.columnconfigure(0, weight=1)

        self.command_list = tk.Listbox(list_frame, height=12, activestyle="dotbox")
        self.command_list.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.command_list.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.command_list.configure(yscrollcommand=scrollbar.set)

        log_frame = ttk.LabelFrame(panel, text="Voice Log", padding=10)
        log_frame.grid(row=5, column=0, sticky="ew")
        self.log_text = tk.Text(log_frame, height=8, wrap=tk.WORD, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def log(self, message: str):
        self.events.put(("log", message))

    def typed_fallback(self, reason: str):
        self.events.put(("log", reason))
        result = {"value": None}
        done = threading.Event()

        def ask():
            dialog = tk.Toplevel(self)
            dialog.title("Type Voice Response")
            dialog.geometry("420x170")
            dialog.transient(self)
            dialog.grab_set()
            ttk.Label(dialog, text=reason, wraplength=380).pack(anchor="w", padx=16, pady=(14, 8))
            entry = ttk.Entry(dialog)
            entry.pack(fill=tk.X, padx=16)
            entry.focus_set()

            def submit():
                result["value"] = entry.get().strip()
                dialog.destroy()
                done.set()

            ttk.Button(dialog, text="Submit", command=submit).pack(pady=12)
            dialog.bind("<Return>", lambda _event: submit())

        self.events.put(("call", ask))
        done.wait()
        return result["value"] or ""

    def start_voice_flow(self):
        threading.Thread(target=self._voice_flow, daemon=True).start()

    def test_microphone(self):
        threading.Thread(target=self._test_microphone, daemon=True).start()

    def open_microphone_settings(self):
        if not sys.platform.startswith("win"):
            messagebox.showinfo("Microphone settings", "Open your system microphone settings and allow desktop apps.")
            return
        try:
            subprocess.Popen(["cmd", "/c", "start", "ms-settings:privacy-microphone"], shell=False)
        except Exception as exc:
            messagebox.showerror("Microphone settings", f"Could not open settings: {exc}")

    def _test_microphone(self):
        test_choices = ["main gate", "library", "yes", "no", "admin", "canteen"]
        heard = self.voice.listen(
            "Microphone test. When you hear listening, say main gate once.",
            self.typed_fallback,
            test_choices,
            attempts=1,
        )
        if heard:
            self.voice.speak(f"I heard {heard}. Microphone test passed.")
            self.events.put(("status", f"Microphone test heard: {heard}"))
        else:
            self.events.put(("status", "Microphone test did not hear a command. Check Windows microphone settings."))

    def _voice_flow(self):
        self.events.put(("status", "Listening for starting point..."))
        start = self._ask_for_place("What is your starting point?")
        if not start:
            self.events.put(("status", "Start cancelled."))
            return

        self.start_node = start
        self.events.put(("draw", None))
        self.events.put(("status", f"Start confirmed: {start.label}. Listening for destination..."))

        end = self._ask_for_place("What is your ending point?")
        if not end:
            self.events.put(("status", "Destination cancelled."))
            return
        if end.id == start.id:
            self.voice.speak("Starting point and ending point are same. Please start again.")
            self.events.put(("status", "Start and destination cannot be the same."))
            return

        self.end_node = end
        self._calculate_route()
        self.events.put(("draw", None))
        self.events.put(("commands", None))
        self.voice.speak(f"Route ready from {start.label} to {end.label}.")
        self.events.put(("status", f"Route ready: {start.label} to {end.label}."))

    def _ask_for_place(self, prompt: str) -> Node | None:
        place_choices = sorted(self.map.alias_lookup)
        yes_no_choices = sorted(YES_WORDS | NO_WORDS)
        while True:
            spoken = self.voice.listen(prompt, self.typed_fallback, place_choices)
            if not spoken:
                self.voice.speak("I did not hear a location. Please try again.")
                continue
            node = self.map.match_place(spoken)
            if not node:
                self.voice.speak("I could not find that place on the campus map. Please say it again.")
                continue
            self.events.put(("candidate", node))
            answer = self.voice.listen(
                f"Did you say {node.label}? Say yes or no.",
                self.typed_fallback,
                yes_no_choices,
            )
            cleaned = re.sub(r"[^a-z ]+", "", answer.lower()).strip()
            if any(word in cleaned.split() or word == cleaned for word in YES_WORDS):
                self.voice.speak(f"Confirmed {node.label}.")
                return node
            if any(word in cleaned.split() or word == cleaned for word in NO_WORDS):
                self.voice.speak("Okay, please say the location again.")
                continue
            self.voice.speak("Please answer yes or no.")

    def _calculate_route(self):
        self.route, total = self.map.shortest_path(self.start_node.id, self.end_node.id)
        if not self.route:
            self.commands = ["No route found."]
            return
        self.commands = []
        for index, node_id in enumerate(self.route[:-1]):
            current = self.map.nodes[node_id]
            next_node = self.map.nodes[self.route[index + 1]]
            segment = self._segment_distance(current.id, next_node.id)
            direction = self._direction(current, next_node)
            self.commands.append(
                f"{index + 1}. Go {direction} from {current.label} to {next_node.label} for {segment:.0f} meters."
            )
        self.commands.append(f"Arrive at {self.end_node.label}. Total distance {total:.0f} meters.")
        self.audio_index = 0

    def _segment_distance(self, from_id: str, to_id: str) -> float:
        for edge in self.map.edges:
            forward = edge["from"] == from_id and edge["to"] == to_id
            backward = edge.get("bidirectional") and edge["to"] == from_id and edge["from"] == to_id
            if forward or backward:
                return edge["distance"]
        return 0

    def _direction(self, current: Node, next_node: Node) -> str:
        dx = next_node.x - current.x
        dy = next_node.y - current.y
        if abs(dx) > abs(dy):
            return "right" if dx > 0 else "left"
        return "down" if dy > 0 else "up"

    def play_audio_commands(self):
        if not self.commands:
            messagebox.showinfo("No route", "Create a route first.")
            return
        threading.Thread(target=self._play_commands, daemon=True).start()

    def _play_commands(self):
        for index, command in enumerate(self.commands):
            self.audio_index = index
            self.events.put(("highlight", index))
            self.events.put(("status", f"Playing command {index + 1} of {len(self.commands)}."))
            spoken = re.sub(r"^\d+\.\s*", "", command)
            self.voice.speak_navigation(spoken)
            time.sleep(0.35)
        self.events.put(("status", "All navigation alerts played."))

    def _draw_map(self):
        self.canvas.delete("all")
        width = max(self.canvas.winfo_width(), 700)
        height = max(self.canvas.winfo_height(), 600)
        xs = [node.x for node in self.map.nodes.values()]
        ys = [node.y for node in self.map.nodes.values()]
        margin = 70
        self.scale = min((width - margin * 2) / (max(xs) - min(xs)), (height - margin * 2) / (max(ys) - min(ys)))
        self.offset_x = margin - min(xs) * self.scale
        self.offset_y = margin - min(ys) * self.scale

        self.canvas.create_rectangle(0, 0, width, height, fill="#dfe9e5", outline="")
        self.canvas.create_polygon(
            self._pt(70, 600), self._pt(235, 520), self._pt(420, 590),
            self._pt(640, 545), self._pt(700, 640), self._pt(70, 640),
            fill="#a7d7e8", outline=""
        )

        for edge in self.map.edges:
            a = self.map.nodes[edge["from"]]
            b = self.map.nodes[edge["to"]]
            self.canvas.create_line(*self._pt(a.x, a.y), *self._pt(b.x, b.y), fill="#f0c775", width=15, capstyle=tk.ROUND)
            self.canvas.create_line(*self._pt(a.x, a.y), *self._pt(b.x, b.y), fill="#f8e5b6", width=7, capstyle=tk.ROUND)

        if self.route:
            points = []
            for node_id in self.route:
                node = self.map.nodes[node_id]
                points.extend(self._pt(node.x, node.y))
            self.canvas.create_line(*points, fill="#0f766e", width=8, capstyle=tk.ROUND, joinstyle=tk.ROUND)

        for node in self.map.nodes.values():
            x, y = self._pt(node.x, node.y)
            fill = "#ffffff"
            outline = "#0f766e"
            radius = 10
            if self.start_node and node.id == self.start_node.id:
                fill = "#16a34a"
                outline = "#ffffff"
                radius = 13
            elif self.end_node and node.id == self.end_node.id:
                fill = "#dc2626"
                outline = "#ffffff"
                radius = 13
            self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill=fill, outline=outline, width=3)
            self.canvas.create_text(x + 14, y - 16, text=node.label, anchor="w", fill="#16211f", font=("Segoe UI", 10, "bold"))

    def _pt(self, x: float, y: float):
        return x * self.scale + self.offset_x, y * self.scale + self.offset_y

    def _process_events(self):
        while True:
            try:
                event, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if event == "log":
                self.log_text.configure(state=tk.NORMAL)
                self.log_text.insert(tk.END, f"{payload}\n")
                self.log_text.see(tk.END)
                self.log_text.configure(state=tk.DISABLED)
            elif event == "status":
                self.status.set(payload)
            elif event == "draw":
                self._draw_map()
            elif event == "candidate":
                self.status.set(f"Recognized: {payload.label}. Waiting for confirmation.")
                self._draw_map()
                x, y = self._pt(payload.x, payload.y)
                self.canvas.create_oval(x - 18, y - 18, x + 18, y + 18, outline="#f59e0b", width=4)
            elif event == "commands":
                self.command_list.delete(0, tk.END)
                for command in self.commands:
                    self.command_list.insert(tk.END, command)
            elif event == "highlight":
                self.command_list.selection_clear(0, tk.END)
                self.command_list.selection_set(payload)
                self.command_list.see(payload)
            elif event == "call":
                payload()
        self.after(100, self._process_events)


if __name__ == "__main__":
    if not MAP_FILE.exists():
        messagebox.showerror("Missing map", f"Could not find {MAP_FILE}")
        raise SystemExit(1)
    app = NavigatorApp()
    app.mainloop()
