# Build tools for splat/ on the Jetson Nano: JetPack 4.6 (L4T r32.7, Ubuntu 18.04), gcc 7.5,
# CMake and nlohmann/json. jetson/splat.sh builds this image and compiles splat in it.
#
# CUDA isn't in the image. On JetPack 4 the NVIDIA container runtime mounts the host's
# CUDA 10.2 into any container started with --runtime nvidia, so splat.sh compiles and runs
# splat with `docker run`, never during `docker build`, and Docker's default runtime can stay
# as it is.
#
# BASE is the image to start from: L4T for the Nano. jetson/check.sh builds the same file on a
# PC from Ubuntu 18.04 with CUDA 10.2, and from arm64 Ubuntu 18.04 under emulation.
ARG BASE=nvcr.io/nvidia/l4t-base:r32.7.1
FROM ${BASE}

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        python3-pip \
        python3-setuptools \
        xz-utils \
    && rm -rf /var/lib/apt/lists/*

# splat needs CMake 3.18 or newer and Ubuntu 18.04 has 3.10. The CMake wheels need a newer pip
# than 18.04's 9.0: pip 21.3 is the last for its Python 3.6, and CMake 3.28 the last wheel.
RUN python3 -m pip install --no-cache-dir --upgrade "pip<22" \
    && python3 -m pip install --no-cache-dir "cmake==3.28.4"

# nlohmann/json 3.11, installed so that CMake finds it instead of downloading it into every
# new build directory (Ubuntu 18.04's package is 2.1, too old).
ADD https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz /tmp/json.tar.xz
RUN echo "d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d  /tmp/json.tar.xz" \
        | sha256sum -c - \
    && tar -xJf /tmp/json.tar.xz -C /tmp \
    && cmake -S /tmp/json -B /tmp/json-build -DJSON_BuildTests=OFF \
    && cmake --install /tmp/json-build \
    && rm -rf /tmp/json /tmp/json-build /tmp/json.tar.xz

ENV PATH=/usr/local/cuda/bin:${PATH}
WORKDIR /data
