"""Checks that every kernel in a CUDA library can launch on the Jetson Nano's GPU.

The Nano's Maxwell (sm_53) has 32K registers per block, half of what desktop GPUs have, so a
kernel that runs on a PC can fail on the Nano with "too many resources requested for launch".
This reads each kernel's registers and shared memory from the library's sm_53 code, and its
block size from the __launch_bounds__ in its PTX. Kernels without launch bounds are taken to
run in blocks of 256 threads, the most splat launches.

    python3 kernels.py LIBRARY.a     (needs cuobjdump on the PATH)
"""
import re
import subprocess
import sys

REGS_PER_BLOCK = 32 * 1024  # sm_53
SHARED_PER_BLOCK = 48 * 1024
REG_UNIT = 256  # registers are handed out per warp, 256 at a time
DEFAULT_BLOCK = 256


def cuobjdump(*args):
    return subprocess.run(["cuobjdump"] + list(args), check=True, stdout=subprocess.PIPE,
                          universal_newlines=True).stdout


def demangle(names):
    out = subprocess.run(["c++filt"], input="\n".join(names), stdout=subprocess.PIPE,
                         universal_newlines=True).stdout.splitlines()
    return dict(zip(names, out))


def main(lib):
    usage = {}
    for name, fields in re.findall(r"Function (\S+):\n\s+(REG:.*)", cuobjdump("-res-usage", "-arch", "sm_53", lib)):
        usage[name] = {k: int(v) for k, v in re.findall(r"(\w+):(\d+)", fields)}
    if not usage:
        sys.exit("no sm_53 code in " + lib)
    block = {}
    for name, after in re.findall(r"\.entry\s+(\S+)\(.*?\)([^{]*)\{", cuobjdump("-ptx", lib), re.S):
        m = re.search(r"\.maxntid\s+(\d+)(?:,\s*(\d+))?(?:,\s*(\d+))?", after)
        if m:
            block[name] = int(m.group(1)) * int(m.group(2) or 1) * int(m.group(3) or 1)

    pretty = demangle(sorted(usage))
    bad = 0
    print("regs  threads  regs/block  shared  kernel (sm_53)")
    for name in sorted(usage, key=lambda n: -usage[n]["REG"]):
        regs, shared = usage[name]["REG"], usage[name]["SHARED"]
        threads = block.get(name, DEFAULT_BLOCK)
        warps = (threads + 31) // 32
        per_block = warps * -(-regs * 32 // REG_UNIT) * REG_UNIT
        ok = per_block <= REGS_PER_BLOCK and shared <= SHARED_PER_BLOCK
        bad += not ok
        label = re.sub(r"^void |\(anonymous namespace\)::", "", pretty[name])
        label = re.sub(r"<(.*)>", lambda m: m.group(0) if len(m.group(1)) < 8 else "<...>",
                       re.sub(r"\(.*", "", label))
        print("{:4d}  {:7d}{} {:10d}  {:6d}  {}{}".format(regs, threads, " " if name in block else "*",
                                                        per_block, shared, label,
                                                        "" if ok else "   <-- too big for the Nano"))
    print("* no __launch_bounds__, so {} threads assumed".format(DEFAULT_BLOCK))
    if bad:
        sys.exit("{} kernel(s) can't launch on the Nano's GPU".format(bad))
    print("All {} kernels fit the Nano's limits: {} registers and {} KB of shared memory per block"
          .format(len(usage), REGS_PER_BLOCK, SHARED_PER_BLOCK // 1024))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
