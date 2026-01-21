FROM python:3.9-slim

# Install system dependencies for OpenCV and GPIO
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libgpiod2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY src/ ./src/

# Command to run the application
CMD ["python", "src/main.py"]
