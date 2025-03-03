# Base image with CUDA support
FROM nvidia/cuda:12.4.1-base-ubuntu22.04

# Configure environment variables
ARG PYTHON_VERSION=3.10
ENV DEBIAN_FRONTEND=noninteractive
ENV MUJOCO_GL="egl"
ENV PATH="/opt/venv/bin:$PATH"
ENV CONDA_DIR="/root/miniconda3"
ENV PATH="$CONDA_DIR/bin:$PATH"
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Install dependencies, Python, and system utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-lfs wget bzip2 \
    libglib2.0-0 libgl1-mesa-glx libegl1-mesa ffmpeg \
    speech-dispatcher libgeos-dev x11-apps \
    python${PYTHON_VERSION}-dev python${PYTHON_VERSION}-venv \
    && ln -s /usr/bin/python${PYTHON_VERSION} /usr/bin/python \
    && python -m venv /opt/venv \
    && apt-get clean && rm -rf /var/lib/apt/lists/* \
    && echo "source /opt/venv/bin/activate" >> /root/.bashrc

# Install Miniconda
RUN mkdir -p $CONDA_DIR \
    && wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh \
    && bash /tmp/miniconda.sh -b -u -p $CONDA_DIR \
    && rm /tmp/miniconda.sh \
    && conda init bash

    # Install LeRobot
RUN git lfs install
RUN git clone https://github.com/huggingface/lerobot.git /lerobot
WORKDIR /lerobot
RUN pip install --upgrade --no-cache-dir pip

# Create and activate Conda environment
RUN conda create -y -n lerobot python=${PYTHON_VERSION} \
    && echo "source ~/miniconda3/bin/activate && conda activate lerobot" >> /root/.bashrc

# Install LeRobot and dependencies in Conda environment
COPY . /lerobot
WORKDIR /lerobot
RUN bash -c "source ~/miniconda3/bin/activate && conda activate lerobot && pip install -e '.[intelrealsense, dynamixel]'"

# Install FFmpeg and OpenCV via Conda
RUN bash -c "source ~/miniconda3/bin/activate && \
    conda activate lerobot && \
    conda install -y -c conda-forge ffmpeg && \
    pip uninstall -y opencv-python && \
    conda install -y -c conda-forge 'opencv>=4.10.0'"

# Install PyTorch with CUDA 12.1
RUN bash -c "source ~/miniconda3/bin/activate && \
    conda activate lerobot && \
    pip install torch==2.5.1+cu121 torchvision==0.20.1+cu121 torchaudio==2.5.1+cu121 --index-url https://download.pytorch.org/whl/cu121"

# Ensure Conda environment activation in every new shell
SHELL ["/bin/bash", "-c"]

# Enable access to USB devices and graphical display
RUN usermod -aG dialout root

# Default command
CMD ["/bin/bash"]
