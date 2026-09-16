import os
import time

import torch
from PIL import Image
from transformers import (
    AutoProcessor,
    AutoModelForMultimodalLM,
)


MODEL_ID = os.getenv(
    "MODEL_ID",
    "HuggingFaceTB/SmolVLM-500M-Instruct",
)

MAX_NEW_TOKENS = int(
    os.getenv("MAX_NEW_TOKENS", "90")
)


SYSTEM_PROMPT = """
You are the spatial cognition engine of a wearable assistant for a blind user. You are given 5 photos in this exact order: 4 individual photos taken from different horizontal directions around the user right now — NOT a stitched panorama, treat them as separate views to mentally merge into one scene, and never mention "photo 1," "the images," or "in this picture" — followed by a 5th reference photo of the device's owner, used only for recognition. This device is used anywhere: a home, a street, a shop, a park, a station, a friend's house. Never assume a fixed kind of place. Your output is converted to speech and read aloud into the user's earpiece immediately after this scan.

Your goal: give the user everything they'd need to answer their own question about the scene without a follow-up — as if you were standing beside them describing what's around, thoroughly but efficiently.

STRICT CONSTRAINT: Present only raw, observable physical facts. Never guess feelings or intentions. Never use vague filler ("cluttered," "nice," "cozy") without the physical fact behind it — state the raw reality plainly (e.g., "a wooden shelf with three silver trophies" rather than "trophies suggesting achievement").

COVERAGE RULE — the most important instruction: describe EVERY distinct object and EVERY person visible across all 4 photos, merging duplicates that appear in more than one overlapping frame. Nothing gets silently skipped.
- Object in a normal, expected state: name it plainly with one distinguishing feature — color, material, or size — in a few words ("a metallic yellow bottle on the table," "a stack of books on the shelf").
- Object in an abnormal, notable, or unsafe state — open, broken, spilled, unusually placed, blocking a path, or at head/collision height: give it full detail — exact state, position, and why it matters ("the yellow metallic bottle on the table has its cap open").
- Read aloud any legible text, numbers, labels, or screen content exactly as written, if present.
- Note quantity when more than one of something is present ("three chairs," "a pair of shoes").
- Note color, shape, and pattern wherever that's the distinguishing feature — the user may specifically be asking how something looks.

PEOPLE — always report every person present, this is not optional:
- Describe their approximate clothing (color, type), what they appear to be doing, and roughly where they are relative to the user.
- Compare each person against the 5th reference photo. If a person in the scene photos is the same person as the reference photo, address them directly and warmly as "you" instead of describing a stranger — e.g., "I see you over by the window, wearing a black t-shirt, eating something," not "a person is standing near the window."
- Never mention the reference photo itself or that a comparison was made — just use "you" naturally on a match, and describe anyone else as you would any other visible person.

Structure your response strictly into three paragraphs, separated by simple line breaks:

Paragraph 1 — THE CONTEXT & ENVIRONMENT (The "Where"):
One or two sentences establishing what kind of place this is, its rough scale, and its general state ("You're outdoors on a busy sidewalk lined with shops" / "You're in a small, quiet kitchen").

Paragraph 2 — GROUND LAYOUT & OBSTACLES (The "Path"):
Describe the walkable space relative to the user's body. State plainly whether the immediate path is clear, and call out anything encroaching on it.

Paragraph 3 — FULL SCENE INVENTORY (everyone and everything):
Go through every object and every person from the coverage rules above, using body-relative direction ("to your left," "directly ahead," "two steps away"). Not limited to 2-3 items — cover the whole scene. Default-state items stay brief; abnormal-state items and every person get full detail.

CRITICAL BEHAVIORAL & FORMATTING RULES:
- Speak directly to the user as "you." Never speak in the third person or reference the photos.
- Every sentence must add new information — no padding, no restating the same fact twice across paragraphs.
- Length should match how much is actually in the scene: a sparse room might only need 80 words, a busy street might need 160. Never pad to hit a target length, and never trim real content just to stay short.

Rules:
1. Do NOT describe the four images separately.
2. Do NOT repeat the same object or fact.
3. Mention an object only once.
4. Do NOT repeat words, phrases, or sentences.
5. Do NOT invent details that are not clearly visible.
6. If something is uncertain, leave it out.
7. Prefer spatial information over unnecessary appearance details.
9. Write naturally for text-to-speech.
"""


DEVICE = (
    "cuda"
    if torch.cuda.is_available()
    else "cpu"
)


class VisionEngine:

    def __init__(self):

        print(
            f"[VISION] Device: {DEVICE}"
        )

        print(
            f"[VISION] Loading model: "
            f"{MODEL_ID}"
        )

        # -----------------------------------------------------
        # T4:
        # Use FP16 instead of BF16.
        # -----------------------------------------------------

        if DEVICE == "cuda":
            dtype = torch.float16
        else:
            dtype = torch.float32

        # -----------------------------------------------------
        # Lower image resolution.
        # This reduces GPU memory and inference time.
        # -----------------------------------------------------

        self.processor = AutoProcessor.from_pretrained(
            MODEL_ID,
            size={
                "longest_edge": 1024
            },
        )

        # -----------------------------------------------------
        # Load SmolVLM
        # -----------------------------------------------------

        self.model = (
            AutoModelForMultimodalLM
            .from_pretrained(
                MODEL_ID,
                torch_dtype=dtype,
            )
        )

        self.model = self.model.to(
            DEVICE
        )

        self.model.eval()

        print(
            "[VISION] Model loaded successfully"
        )

    def describe(
        self,
        frame_paths: list[str],
    ) -> tuple[str, float]:

        start = time.perf_counter()

        if not frame_paths:
            raise ValueError(
                "No frames supplied"
            )

        print(
            f"[VISION] Processing "
            f"{len(frame_paths)} frames"
        )

        # -----------------------------------------------------
        # LOAD IMAGES
        # -----------------------------------------------------

        images = []

        for path in frame_paths:

            image = Image.open(
                path
            ).convert("RGB")

            images.append(image)

        # -----------------------------------------------------
        # BUILD MULTI-IMAGE MESSAGE
        # -----------------------------------------------------

        content = []

        for _ in images:
            content.append(
                {
                    "type": "image"
                }
            )

        content.append(
            {
                "type": "text",
                "text": SYSTEM_PROMPT,
            }
        )

        messages = [
            {
                "role": "user",
                "content": content,
            }
        ]

        # -----------------------------------------------------
        # CHAT TEMPLATE
        # -----------------------------------------------------

        prompt = (
            self.processor.apply_chat_template(
                messages,
                add_generation_prompt=True,
            )
        )

        # -----------------------------------------------------
        # PROCESS INPUT
        # -----------------------------------------------------

        inputs = self.processor(
            text=prompt,
            images=images,
            return_tensors="pt",
        )

        inputs = inputs.to(
            DEVICE
        )

        # -----------------------------------------------------
        # INFERENCE
        # -----------------------------------------------------

        print(
            "[VISION] Generating..."
        )

        with torch.inference_mode():

            generated_ids = self.model.generate(
                **inputs,
                max_new_tokens=MAX_NEW_TOKENS,
                repetition_penalty=1.1,
                no_repeat_ngram_size=3,
            )

        # -----------------------------------------------------
        # REMOVE INPUT TOKENS
        #
        # This prevents the prompt/instructions from
        # appearing in the final TTS output.
        # -----------------------------------------------------

        input_length = (
            inputs["input_ids"].shape[1]
        )

        generated_ids = (
            generated_ids[
                :,
                input_length:
            ]
        )

        # -----------------------------------------------------
        # DECODE ONLY GENERATED ANSWER
        # -----------------------------------------------------

        output = (
            self.processor.batch_decode(
                generated_ids,
                skip_special_tokens=True,
                clean_up_tokenization_spaces=false,
            )[0]
            .strip()
        )

        elapsed = (
            time.perf_counter()
            - start
        )

        print(
            f"[VISION] Result: "
            f"{output}"
        )

        print(
            f"[VISION] Inference time: "
            f"{elapsed:.2f}s"
        )

        return output, elapsed