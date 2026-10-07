# Ubuntu 18.04 with CUDA 10.2 and gcc 7.5, JetPack 4.6's compilers, on an x86 PC. check.sh
# builds jetson/Dockerfile on top of it to compile splat for the Nano's GPU (sm_53).
#
# NVIDIA's CUDA 10.2 images are gone from Docker Hub, so the toolkit is copied out of
# PyTorch's 1.9.0 image for CUDA 10.2, which is built on NVIDIA's
# nvidia/cuda:10.2-cudnn7-devel-ubuntu18.04. Any image with /usr/local/cuda-10.2 will do.
ARG CUDA_IMAGE=pytorch/pytorch:1.9.0-cuda10.2-cudnn7-devel
ARG UBUNTU=ubuntu:18.04

FROM ${CUDA_IMAGE} AS cuda
# Keep the compiler, the headers and the runtime; drop the libraries splat doesn't use.
RUN cd /usr/local/cuda-10.2 \
    && rm -rf doc libnsight libnvvp nsight-compute* nsight-systems* nsightee_plugins extras/CUPTI \
    && cd targets/x86_64-linux/lib \
    && rm -f libcufft* libcurand* libcusolver* libcusparse* libnpp* libnvjpeg* libnvgraph* \
             libcupti* libnvperf* libnvrtc* libaccinj* libcuinj* liblapack* libmetis*

FROM ${UBUNTU}
COPY --from=cuda /usr/local/cuda-10.2 /usr/local/cuda-10.2
RUN ln -s cuda-10.2 /usr/local/cuda
ENV PATH=/usr/local/cuda/bin:${PATH}
