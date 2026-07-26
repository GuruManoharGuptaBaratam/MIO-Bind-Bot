idf_component_register(
    SRCS "src/audio_feedback.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_timer esp_ringbuf
      EMBED_FILES
        "sdmiss.adpcm"    "sdbad.adpcm"     "wififail.adpcm"
        "sdok.adpcm"      "wwok.adpcm"      "wwretry.adpcm"
        "cnt3.adpcm"      "cnt2.adpcm"      "cnt1.adpcm"    "kansei.adpcm"    "kiroku.adpcm"
        "ibasho.adpcm"    "cmdunk.adpcm"    "cmdtout.adpcm"
        "idle.adpcm"      "wificonn.adpcm"  "wifidisc.adpcm"
        "scenepr.adpcm"   "scenedn.adpcm"   "recstart.adpcm"
        "recsave.adpcm"
)