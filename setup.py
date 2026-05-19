from setuptools import setup, Extension
from Cython.Build import cythonize
import os
import numpy
import json 
from pathlib import Path

# Print python version
print(f"Python version: {os.sys.version}")

curr_path =  os.path.dirname(os.path.abspath(__file__))
# SET ENV VALUES
with open(os.path.join(curr_path, ".env")) as f:
    for line in f:
        key, value = line.strip().split("=")
        os.environ[key] = os.path.join(curr_path, value.strip('"').strip("'"))
        
SDK_ROOT = Path(os.environ["RS_DRIVER_REPO_PATH"])

include_dirs_hesai = [
    str(SDK_ROOT),
    str(SDK_ROOT / "driver"),
    str(SDK_ROOT / "libhesai"),
    str(SDK_ROOT / "libhesai" / "Common"),
    str(SDK_ROOT / "libhesai" / "Common" / "include"),
    str(SDK_ROOT / "libhesai" / "Container"),
    str(SDK_ROOT / "libhesai" / "Container" / "include"),
    str(SDK_ROOT / "libhesai" / "Container" / "src"),
    str(SDK_ROOT / "libhesai" / "Lidar"),
    str(SDK_ROOT / "libhesai" / "Logger"),
    str(SDK_ROOT / "libhesai" / "Logger" / "include"),
    str(SDK_ROOT / "libhesai" / "Logger" / "src"),
    str(SDK_ROOT / "libhesai" / "PtcClient"),
    str(SDK_ROOT / "libhesai" / "PtcClient" / "include"),
    str(SDK_ROOT / "libhesai" / "PtcParser"),
    str(SDK_ROOT / "libhesai" / "PtcParser" / "include"),
    str(SDK_ROOT / "libhesai" / "SerialClient"),
    str(SDK_ROOT / "libhesai" / "SerialClient" / "include"),
    str(SDK_ROOT / "libhesai" / "Source"),
    str(SDK_ROOT / "libhesai" / "Source" / "include"),
    str(SDK_ROOT / "libhesai" / "UdpParser"),
    str(SDK_ROOT / "libhesai" / "UdpParser" / "include"),
    str(SDK_ROOT / "libhesai" / "UdpParser" / "src"),
    str(SDK_ROOT / "libhesai" / "UdpProtocol"),
]


ext_modules = [
    Extension(
        name="HesaiLidar_driver_wrapper",
        sources=[
            "src/core.cpp", 
            "src/HesaiLidarDriverWrapper.pyx", 
            "src/logs_redirection.cpp", 
            "src/HesaiDriver.cpp",
            
            str(SDK_ROOT / "libhesai" / "Logger" / "src" / "logger.cc"),
            str(SDK_ROOT / "libhesai" / "Common" / "src" / "plat_utils.cc"),
            str(SDK_ROOT / "libhesai" / "SerialClient" / "src" / "serial_client.cc"),
            str(SDK_ROOT / "libhesai" / "Source" / "src" / "tcp_source.cc"),
            str(SDK_ROOT / "libhesai" / "Source" / "src" / "serial_source.cc"),
            str(SDK_ROOT / "libhesai" / "Source" / "src" / "pcap_source.cc"),
            str(SDK_ROOT / "libhesai" / "Source" / "src" / "socket_source.cc"),
            str(SDK_ROOT / "libhesai" / "Source" / "src" / "pcap_saver.cc"),
            str(SDK_ROOT / "libhesai" / "PtcClient" / "src" / "ptc_client.cc"),
            str(SDK_ROOT / "libhesai" / "PtcClient" / "src" / "tcp_client.cc"),
            # str(SDK_ROOT / "libhesai" / "PtcClient" / "src" / "tcp_ssl_client.cc"),
            str(SDK_ROOT / "libhesai" / "Source" / "src" / "source.cc"),
            str(SDK_ROOT / "libhesai" / "PtcParser" / "ptc_parser.cc"),
            str(SDK_ROOT / "libhesai" / "PtcParser" / "src" / "general_ptc_parser.cc"),
            str(SDK_ROOT / "libhesai" / "PtcParser" / "src" / "ptc_1_0_parser.cc"),
            str(SDK_ROOT / "libhesai" / "PtcParser" / "src" / "ptc_2_0_parser.cc"),
            ],
        language="c++",
        include_dirs=[
            os.path.join(os.environ["NPCAP_SDK_PATH"], "Include"),
            os.path.join(os.environ["NPCAP_SDK_PATH"], "Include", "pcap"),
            numpy.get_include(),
        ] + include_dirs_hesai,
        library_dirs=[os.path.join(os.environ["NPCAP_SDK_PATH"], "Lib", "x64")],
        libraries=["wpcap", "Packet", "Ws2_32"],
        extra_compile_args=[],
    )
]

setup(
    name="HesaiLidar_driver_wrapper",
    ext_modules=cythonize(ext_modules),
    # Map the root package to the `src` directory so built extensions
    # and python modules end up in `src/` instead of the project root.
    # This prevents a duplicate .pyd from being created next to setup.py
    # when building in-place or during a normal build.
    package_dir={"": "src"},
    options={"build": {"build_lib": os.path.join(curr_path, "src")}},
)

# Write dependency commit hash to txt file
import subprocess
def write_git_commit_hash(repo_path: str, output_file: str):

    if not os.path.exists(repo_path) or not os.path.exists(os.path.join(repo_path, ".git")):
        raise ValueError(f"The path '{repo_path}' is not a valid Git repository.")
    
    try:
        commit_hash = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo_path),
            text=True
        ).strip()
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to get git commit hash: {e}")

    # Write the hash to the output file
    with open(output_file, "w") as f:
        f.write(commit_hash + "\n")

    print(f"Commit hash '{commit_hash}' written to {output_file}")

write_git_commit_hash(curr_path + "/../HesaiLidar_SDK_2.0", curr_path + "/rs_driver_dep.txt")