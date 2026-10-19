# Campus Offline Voice Navigator

Tkinter desktop navigation app using the supplied campus map graph.

## Run

```powershell
python campus_navigator.py
```

If `python` is not on PATH, try:

```powershell
py campus_navigator.py
```

Or double-click `run_navigator.bat`.

## Voice support

The app can speak using Windows speech synthesis or `pyttsx3`.

Microphone input first tries Vosk, which works offline and does not use the
Windows speech recognizer popup path.

Install the Python voice packages:

```powershell
pip install -r requirements.txt
```

Then download an English Vosk model and put the extracted folder beside
`campus_navigator.py` as either:

- `model`
- `vosk-model-small-en-us-0.15`

If speech recognition is not configured or the microphone fails, the app opens a
small text fallback dialog whenever it needs your spoken answer.
## Vosk Model Setup

1. Download the Vosk small English model.
2. Extract the downloaded ZIP.
3. Place the extracted folder inside the offline-navigation directory.

Expected structure:

offline-navigation/
├── campus_navigator.py
├── campus_map.json
├── requirements.txt
└── vosk-model-small-en-us-0.15/
