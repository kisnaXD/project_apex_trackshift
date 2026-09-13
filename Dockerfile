FROM python:3.11-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    curl \
    gfortran \
    liblapack-dev \
    libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -U pip setuptools wheel \
    && pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 8501 8765 8766

ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/app
ENV STREAMLIT_SERVER_PORT=8501
ENV STREAMLIT_SERVER_ADDRESS=0.0.0.0
ENV STREAMLIT_SERVER_HEADLESS=true
ENV FASTF1_CACHE_DIR=/app/telemetry_logs/fastf1_cache
ENV F1SIM_LOG_DIR=/app/telemetry_logs

HEALTHCHECK CMD curl --fail http://localhost:8501/_stcore/health || exit 1

ENTRYPOINT ["streamlit", "run", "streamlit_app/app.py"]
