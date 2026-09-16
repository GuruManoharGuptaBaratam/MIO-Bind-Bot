import os
import struct
import tempfile
import time

from dotenv import load_dotenv
from fastapi import FastAPI, File, Header, HTTPException, UploadFile
from fastapi.responses import Response

from app.vision import VisionEngine
from app.tts import synthesize_pcm16


load_dotenv()

MIO_SHARED_SECRET = os.getenv("MIO_SHARED_SECRET", "")

app = FastAPI(title="MIO Inference Server")

print("[MIO] Loading vision engine...")
vision_engine = VisionEngine()
print("[MIO] Vision engine ready")


@app.get("/health")
def health():
    return {
        "status": "ok",
        "service": "mio-inference-server",
    }


@app.post("/kansei")
async def kansei(
    frame_0: UploadFile = File(...),
    frame_1: UploadFile = File(...),
    frame_2: UploadFile = File(...),
    frame_3: UploadFile = File(...),
    user_ref: UploadFile | None = File(None),
    x_mio_key: str | None = Header(None),
):
    start_total = time.perf_counter()

    # ---------------------------------------------------------
    # AUTHENTICATION
    # ---------------------------------------------------------

    if not MIO_SHARED_SECRET:
        raise HTTPException(
            status_code=500,
            detail="MIO_SHARED_SECRET is not configured",
        )

    if x_mio_key != MIO_SHARED_SECRET:
        raise HTTPException(
            status_code=401,
            detail="Invalid MIO key",
        )

    # ---------------------------------------------------------
    # SAVE UPLOADED FRAMES
    # ---------------------------------------------------------

    uploaded_frames = [
        frame_0,
        frame_1,
        frame_2,
        frame_3,
    ]

    with tempfile.TemporaryDirectory() as tmp:

        frame_paths = []

        for i, upload in enumerate(uploaded_frames):

            data = await upload.read()

            if not data:
                raise HTTPException(
                    status_code=400,
                    detail=f"frame_{i} is empty",
                )

            path = os.path.join(
                tmp,
                f"frame_{i}.jpg",
            )

            with open(path, "wb") as f:
                f.write(data)

            frame_paths.append(path)

            print(
                f"[KANSEI] frame_{i}: "
                f"{len(data)} bytes"
            )

        # -----------------------------------------------------
        # VISION
        # -----------------------------------------------------

        print("[KANSEI] Running vision...")

        description, vision_time = vision_engine.describe(
            frame_paths
        )

        print(
            f"[KANSEI] Vision complete: "
            f"{vision_time:.2f}s"
        )

        # -----------------------------------------------------
        # TTS
        # -----------------------------------------------------

        print("[KANSEI] Running TTS...")

        pcm_audio, tts_time = await synthesize_pcm16(
            description
        )

        print(
            f"[KANSEI] TTS complete: "
            f"{tts_time:.2f}s"
        )

    # ---------------------------------------------------------
    # RESPONSE FORMAT
    #
    # [4 bytes] text length, little endian uint32
    # [N bytes] UTF-8 description
    # [remaining] PCM16 mono 8kHz
    # ---------------------------------------------------------

    text_bytes = description.encode("utf-8")

    response = (
        struct.pack("<I", len(text_bytes))
        + text_bytes
        + pcm_audio
    )

    total_time = time.perf_counter() - start_total

    print(
        f"[KANSEI] "
        f"Vision={vision_time:.2f}s | "
        f"TTS={tts_time:.2f}s | "
        f"Total={total_time:.2f}s"
    )

    print(
        f"[KANSEI] "
        f"text={len(text_bytes)} bytes | "
        f"audio={len(pcm_audio)} bytes"
    )

    return Response(
        content=response,
        media_type="application/octet-stream",
    )