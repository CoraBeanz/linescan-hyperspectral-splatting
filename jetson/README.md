# jetson: the Jetson Nano build

Scripts that build and run this repo's code on the Jetson Nano, so it runs the day the Nano is
wired up:

- **splat** ([`../splat`](../splat)), the C++/CUDA line-camera renderer and trainer, built for
  the Nano's GPU (CUDA 10.2, sm_53) in a container that uses JetPack's own CUDA
- **ROS 2** ([`../ros2`](../ros2)), Humble in its Ubuntu 22.04 container, with the image built
  in a way JetPack 4.6's Docker can manage
- a **check for a PC** that builds splat with the Nano's compilers and runs its tests, since
  the Nano isn't always at hand

| File | What it does |
|---|---|
| [`nano_setup.sh`](nano_setup.sh) | One-time setup of the Nano: Docker without sudo, a swap file, the USB port names |
| [`build_all.sh`](build_all.sh) | Day one: builds and tests splat and the ROS 2 packages, plus a short run on the GPU |
| [`splat.sh`](splat.sh) | Builds splat in its container and runs its tests and tools |
| [`ros.sh`](ros.sh) | Builds the ROS 2 image, then runs [`ros2/docker/run.sh`](../ros2/docker/run.sh) |
| [`Dockerfile`](Dockerfile) | splat's image: L4T r32.7 (Ubuntu 18.04), gcc 7.5, CMake 3.28, nlohmann/json |
| [`check.sh`](check.sh), [`check/`](check) | The PC-side check |

## On the Nano

It needs JetPack 4.6.x, the last JetPack for the Nano (L4T r32.7, Ubuntu 18.04, CUDA 10.2).
The SD card image has everything else: Docker and NVIDIA's container runtime.

**Once.** Clone the repo and run the setup:

```bash
git clone https://github.com/CoraBeanz/linescan-hyperspectral-splatting.git
cd linescan-hyperspectral-splatting
jetson/nano_setup.sh        # then log out and back in, so that Docker works without sudo
```

For builds and timings, run the Nano in its 10 W mode: a 5 V 4 A supply on the barrel jack
(with the J48 jumper fitted), then `sudo nvpmodel -m 0`.

**Day one.** `jetson/build_all.sh` builds and tests everything and stops at the first
failure:

1. splat for the GPU, then its unit tests. The GPU tests have to run: if the container
   can't see the GPU, this fails rather than skipping them.
2. A small synthetic dataset, rendered on the GPU and the CPU to compare them (with timings),
   then 300 training steps on the GPU, which prints the time per step and the pose error.
3. The ROS 2 image, `colcon build` and `colcon test`.

The first run downloads the images and compiles everything, so it takes a while; later runs
only redo what changed.

**After that.** splat's builds and data live in `~/linesplat`:

```bash
jetson/splat.sh build                                # after changing splat/
jetson/splat.sh test
jetson/splat.sh splat_synth synth --preset default   # any splat tool runs in ~/linesplat
jetson/splat.sh splat_train synth synth/train
jetson/splat.sh shell                                # a shell in the container
```

ROS 2 works as in [`ros2/README.md`](../ros2/README.md), with `jetson/ros.sh` in place of
`ros2/docker/run.sh` (it takes the same arguments):

```bash
jetson/ros.sh colcon build
jetson/ros.sh ros2 launch so101_scan_bringup scan_arm.launch.py use_mock_hardware:=true mirror:=fake
```

## How it works

**splat's container.** NVIDIA's L4T base image for JetPack 4 has no CUDA in it: the NVIDIA
runtime mounts the Nano's own CUDA 10.2 into each container started with `--runtime nvidia`,
so the CUDA in the container is always the one JetPack installed. The image therefore only
holds the build tools: gcc 7.5 from Ubuntu 18.04, CMake 3.28 (splat needs 3.18, Ubuntu 18.04
has 3.10) and nlohmann/json 3.11. `splat.sh` compiles in `docker run`, never in `docker
build`, so Docker's default runtime can stay as it is. It runs the container as you rather
than root, so `~/linesplat` stays yours, with the groups that own the GPU's device nodes. The
programs link the CUDA runtime statically, so they also run on the Nano outside the
container.

**sm_53.** The build targets only the Nano's GPU, a Maxwell with compute capability 5.3. It
has 32K registers per block where desktop GPUs have 64K, so a kernel can launch on a PC and
fail on the Nano; the PC-side check reads every kernel's registers and shared memory from the
compiled code and checks them against the Nano's limits. splat was already written for
CUDA 10.2: C++14 device code, Thrust rather than CUB, no double-precision atomics, and the
`_sync` warp intrinsics ([`splat/README.md`](../splat/README.md)).

**Memory.** The Nano has 4 GB shared between the CPU and the GPU. `splat.sh` compiles two
files at a time (`LINESPLAT_JOBS=1` if the compiler still runs out), the ROS 2 image builds
two packages at a time, and `nano_setup.sh` adds a 4 GB swap file.

**The ROS 2 image.** Humble needs Ubuntu 22.04, whose C library starts threads and processes
with the `clone3` system call. Docker's seccomp filter, the list of system calls a container
may make, refuses `clone3` in Docker before 20.10.10 (JetPack 4.6 has 20.10.7; `docker
version` shows yours). Newer Dockers answer "no such call" instead, and the C library then
falls back to the old `clone`. `run.sh` sidesteps this by running its containers without the
filter, but `docker build` can't drop it, so on 20.10.7 the image's first step,
`apt-get update`, fails. `ros.sh` checks Docker's version and, if it's older than 20.10.10,
builds the image with a current [BuildKit](https://github.com/moby/buildkit) in a container
instead (the same Dockerfile, with BuildKit's own filter) and loads the result into Docker. On
a newer Docker it's a plain `docker build`.

## Checking from a PC

```bash
jetson/check.sh            # both checks; or jetson/check.sh cuda, jetson/check.sh arm64
```

It uses the Nano's [`Dockerfile`](Dockerfile) and [`splat.sh`](splat.sh) with two other
starting images:

- **cuda**: Ubuntu 18.04 with CUDA 10.2 and gcc 7.5, the Nano's compilers on an x86 PC.
  NVIDIA's CUDA 10.2 images are gone from Docker Hub, so
  [`check/cuda102.Dockerfile`](check/cuda102.Dockerfile) copies the toolkit out of PyTorch's
  1.9.0 image for CUDA 10.2 (4.6 GB to download, once). It builds splat with the CUDA
  rasterizer for sm_53, checks every kernel against the Nano's limits
  ([`check/kernels.py`](check/kernels.py)), and runs the tests; the GPU ones skip, since this
  checks the build, not a GPU.
- **arm64**: arm64 Ubuntu 18.04 under emulation, the Nano's CPU and distro: the CPU build and
  the tests.

It needs Docker, and for arm64 the QEMU emulator registered with the kernel (Docker Desktop
has it; on Linux, `docker run --privileged --rm tonistiigi/binfmt --install arm64`). The builds
go to `build/jetson-check/`.

What only the Nano can show: the GPU tests and timings, the NVIDIA runtime's mounts, and
BuildKit on the Nano's 4.9 kernel.

## If something fails

- **`unknown or invalid runtime name: nvidia`** or **"No CUDA in the container"**: Docker has
  no NVIDIA runtime (`jetson/nano_setup.sh` checks for it), or the runtime didn't mount
  JetPack's CUDA: `/usr/local/cuda/bin/nvcc --version` should work on the Nano itself, and
  `/etc/nvidia-container-runtime/host-files-for-container.d/cuda.csv` should exist.
- **"The GPU isn't visible in the container"**: `ls -l /dev/nvhost-ctrl /dev/nvmap` shows the
  group that owns the GPU; `splat.sh` adds the container to it, so check that the files exist.
- **The compiler is killed** (`cc1plus: fatal error: Killed`, or `nvcc` dying): out of memory,
  so `LINESPLAT_JOBS=1 jetson/splat.sh build`, and check `free -h` shows the swap.
- **`apt-get update` fails while building the ROS 2 image**: Docker is old and `ros.sh` didn't
  notice; `SO101_BUILD_WITH=buildkit jetson/ros.sh image` forces the BuildKit route.
