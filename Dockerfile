#for non cuda
FROM python:3.10-slim

#some env vars
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 DEBIAN_FRONTEND=noninteractive

#working dir
WORKDIR /workspace

#build essentials
RUN apt-get update && apt-get install -y \
    git \
    wget \
    build-essential \
    cmake \
    libgl1-mesa-dev \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Alternative: If you need OpenGL, use these instead
# RUN apt-get update && apt-get install -y \
#     libgl1 \
#     libglx-mesa0 \
#     libgl1-mesa-dri \
#     && apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

COPY openvla/ ./openvla/

#torch for cpu, change this if you want a gpu torch
RUN pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu

#installing vla deps
WORKDIR /workspace/openvla

RUN pip install --no-cache-dir -r requirements-min.txt

# Install additional common dependencies
RUN pip install --no-cache-dir \
    # transformers \
    accelerate \
    datasets \
    # tensorflow \
    # tensorflow-datasets \
    matplotlib \
    opencv-python \
    pillow \
    scipy \
    tqdm \
    einops \
    timm \
    scikit-image \
    imageio \
    imageio-ffmpeg \
    pynput \
    ipykernel \
    jupyter

RUN pip install -e .

#some libero stuff, if needed
RUN if [ -f "experiments/robot/libero/libero_requirements.txt" ]; then \
    pip install --no-cache-dir -r experiments/robot/libero/libero_requirements.txt; \
    fi

#cuda absence flag
ENV CUDA_VISIBLE_DEVICES=""

RUN pip install --no-cache-dir pybullet

RUN mkdir -p /workspace/models



#non root usr
RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/*
ARG USERNAME=kol
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}

#workdir flag
WORKDIR /workspace

CMD ["/bin/bash"]
