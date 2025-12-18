import subprocess
import threading
import queue
import re

def agent_reply(text):
    return f"[AGENT] -> {text}"

q = queue.Queue()

BRACKET_RE = re.compile(r"\[(.*?)\]")

def reader_thread(proc):
    for raw_line in proc.stdout:
        # Remove ANSI escape codes:
        line = re.sub(r"\x1b\\[[0-9;]*[A-Za-z]", "", raw_line).strip()

        if not line:
            continue

        # Case 1: [Texto] like [Música]
        match = BRACKET_RE.fullmatch(line)
        if match:
            text = match.group(1).strip()
            if text:
                q.put(text)
            continue

        # Case 2: Plain transcription text
        q.put(line)


def main():
    whisper_cmd = [
        "/home/pi/Downloads/whisper.cpp/build/bin/whisper-stream",
        "-m", "/home/pi/Downloads/whisper.cpp/models/ggml-base-q5_1.bin",
        "--language", "es",
        "-t", "6",
        "-ac", "512",
    ]

    proc = subprocess.Popen(
        whisper_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    threading.Thread(target=reader_thread, args=(proc,), daemon=True).start()

    print("Agente listo. Escuchando...\n")

    while True:
        text = q.get()
        print("WHISPER:", text)
        print(agent_reply(text))


if __name__ == "__main__":
    main()
