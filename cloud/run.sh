#!/bin/bash

set -e

echo "================================="
echo "Starting MIO Inference Server"
echo "================================="

uvicorn app.main:app \
    --host 0.0.0.0 \
    --port 8000