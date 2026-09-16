import os
import subprocess
import tempfile
import time

import edge_tts


VOICE = os.getenv(
    "TTS_VOICE",
    "en-US-GuyNeural",
)


async def synthesize_pcm16(
    text: str,
) -> tuple[bytes, float]:

    start = time.perf_counter()

    with tempfile.TemporaryDirectory() as tmp:

        mp3_path = os.path.join(
            tmp,
            "speech.mp3",
        )

        pcm_path = os.path.join(
            tmp,
            "speech.pcm",
        )

        # -----------------------------------------------------
        # EDGE TTS
        # -----------------------------------------------------

        communicate = edge_tts.Communicate(
            text,
            VOICE,
        )

        await communicate.save(
            mp3_path
        )

        # -----------------------------------------------------
        # MP3 → PCM16
        #
        # 8000 Hz
        # Mono
        # Signed 16-bit little endian
        # -----------------------------------------------------

        subprocess.run(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-i",
                mp3_path,
                "-ar",
                "8000",
                "-ac",
                "1",
                "-f",
                "s16le",
                pcm_path,
            ],
            check=True,
        )

        with open(
            pcm_path,
            "rb",
        ) as f:

            pcm = f.read()

    elapsed = (
        time.perf_counter()
        - start
    )

    print(
        f"[TTS] Generated "
        f"{len(pcm)} PCM bytes "
        f"in {elapsed:.2f}s"
    )

    return pcm, elapsed