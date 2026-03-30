#!/usr/bin/env python3
"""
MASt3R-SLAM ARM64 Environment Check Script
Checks if all dependencies are correctly installed for ARM64 (AGX Thor)
"""

import sys
import subprocess
import os

# ANSI colors
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
RESET = "\033[0m"
BOLD = "\033[1m"

def success(msg):
    print(f"  {GREEN}✓{RESET} {msg}")

def failure(msg, detail=""):
    print(f"  {RED}✗{RESET} {msg}")
    if detail:
        print(f"    {RED}→ {detail}{RESET}")

def warning(msg, detail=""):
    print(f"  {YELLOW}⚠{RESET} {msg}")
    if detail:
        print(f"    {YELLOW}→ {detail}{RESET}")

def section(title):
    print(f"\n{BOLD}{BLUE}{'='*50}{RESET}")
    print(f"{BOLD}{BLUE}  {title}{RESET}")
    print(f"{BOLD}{BLUE}{'='*50}{RESET}")

def check_python_package(package_name, import_name=None, version_attr="__version__"):
    """Check if a Python package is installed and get its version"""
    if import_name is None:
        import_name = package_name
    try:
        module = __import__(import_name)
        version = getattr(module, version_attr, "unknown")
        success(f"{package_name}: {version}")
        return True
    except ImportError as e:
        failure(f"{package_name}: NOT INSTALLED", str(e))
        return False
    except Exception as e:
        failure(f"{package_name}: ERROR", str(e))
        return False

def check_command(cmd, name=None):
    """Check if a command is available"""
    if name is None:
        name = cmd[0]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        output = result.stdout.strip() or result.stderr.strip()
        success(f"{name}: {output[:60]}")
        return True
    except FileNotFoundError:
        failure(f"{name}: NOT FOUND")
        return False
    except subprocess.TimeoutExpired:
        failure(f"{name}: TIMEOUT")
        return False
    except Exception as e:
        failure(f"{name}: ERROR", str(e))
        return False

def check_file_exists(path, name=None):
    """Check if a file or directory exists"""
    if name is None:
        name = path
    if os.path.exists(path):
        success(f"{name}: EXISTS")
        return True
    else:
        failure(f"{name}: NOT FOUND")
        return False

def main():
    results = {"passed": 0, "failed": 0, "warnings": 0}
    
    print(f"\n{BOLD}MASt3R-SLAM ARM64 Environment Check{RESET}")
    print(f"Python: {sys.version}")
    print(f"Platform: {sys.platform}")
    
    # ==================== CUDA & GPU ====================
    section("CUDA & GPU")
    
    # NVIDIA Driver
    check_command(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"], "NVIDIA Driver")
    
    # CUDA
    check_command(["nvcc", "--version"], "NVCC")
    
    # PyTorch CUDA
    try:
        import torch
        success(f"PyTorch: {torch.__version__}")
        if torch.cuda.is_available():
            success(f"CUDA Available: True (Device: {torch.cuda.get_device_name(0)})")
            success(f"CUDA Version (torch): {torch.version.cuda}")
        else:
            failure("CUDA Available: False")
            results["failed"] += 1
    except ImportError:
        failure("PyTorch: NOT INSTALLED")
        results["failed"] += 1
    except Exception as e:
        failure(f"PyTorch CUDA: ERROR", str(e))
        results["failed"] += 1
    
    # ==================== ROS2 ====================
    section("ROS2")
    
    # Check ROS2 installation
    ros_distro = os.environ.get("ROS_DISTRO", None)
    if ros_distro:
        success(f"ROS_DISTRO: {ros_distro}")
    else:
        warning("ROS_DISTRO not set (run: source /opt/ros/humble/setup.bash)")
        results["warnings"] += 1
    
    check_command(["ros2", "--version"], "ros2 CLI")
    
    # ROS2 packages
    ros_packages = ["rclpy", "sensor_msgs", "geometry_msgs", "std_msgs", "cv_bridge"]
    for pkg in ros_packages:
        try:
            __import__(pkg)
            success(f"ROS2 {pkg}: OK")
        except ImportError:
            failure(f"ROS2 {pkg}: NOT FOUND")
            results["failed"] += 1
    
    # ==================== Core Python Packages ====================
    section("Core Python Packages")
    
    packages = [
        ("numpy", "numpy"),
        ("scipy", "scipy"),
        ("opencv-python", "cv2"),
        ("einops", "einops"),
        ("Pillow", "PIL"),
        ("matplotlib", "matplotlib"),
        ("tqdm", "tqdm"),
        ("PyYAML", "yaml"),
    ]
    
    for pkg_name, import_name in packages:
        if pkg_name == "opencv-python":
            try:
                import cv2
                version = getattr(cv2, "__version__", "unknown")
                success(f"{pkg_name}: {version}")
            except AttributeError as e:
                warning(f"{pkg_name}: WARNING", str(e))
                results["warnings"] += 1
            except Exception as e:
                failure(f"{pkg_name}: ERROR", str(e))
                results["failed"] += 1
            continue

        if not check_python_package(pkg_name, import_name):
            results["failed"] += 1
    
    # ==================== 3D & Vision Packages ====================
    section("3D & Vision Packages")
    
    # Open3D (optional on ARM)
    try:
        import open3d as o3d
        success(f"Open3D: {o3d.__version__}")
    except ImportError:
        warning("Open3D: NOT INSTALLED (optional on ARM)")
        results["warnings"] += 1
    
    # lietorch
    try:
        import lietorch
        success(f"lietorch: OK")
    except ImportError:
        failure("lietorch: NOT INSTALLED")
        results["failed"] += 1
    
    # ==================== MASt3R-SLAM ====================
    section("MASt3R-SLAM")
    
    # MASt3R-SLAM package
    try:
        import mast3r_slam
        success("mast3r_slam: OK")
    except ImportError as e:
        failure("mast3r_slam: NOT INSTALLED", str(e))
        results["failed"] += 1
    
    # CUDA backend
    try:
        import mast3r_slam_backends
        success("mast3r_slam_backends (CUDA): OK")
    except ImportError as e:
        failure("mast3r_slam_backends (CUDA): NOT INSTALLED", str(e))
        results["failed"] += 1
    
    # Checkpoints
    checkpoint_path = "/workspace/MASt3R-SLAM/checkpoints"
    if os.path.exists(checkpoint_path):
        checkpoints = os.listdir(checkpoint_path)
        if checkpoints:
            success(f"Checkpoints: {len(checkpoints)} found")
        else:
            warning("Checkpoints directory empty")
            results["warnings"] += 1
    else:
        warning(f"Checkpoints directory not found: {checkpoint_path}")
        results["warnings"] += 1
    
    # ==================== Optional Packages ====================
    section("Optional Packages")
    
    # pyrealsense2 (optional on ARM)
    try:
        import pyrealsense2
        try:
            version = pyrealsense2.__version__
        except AttributeError:
            version = "installed (version unavailable)"
        success(f"pyrealsense2: {version}")
    except ImportError:
        warning("pyrealsense2: NOT INSTALLED (optional on ARM)")
        results["warnings"] += 1
    
    # imgui
    try:
        import imgui
        success(f"imgui: OK")
    except ImportError:
        warning("imgui: NOT INSTALLED (needed for GUI)")
        results["warnings"] += 1
    
    # glfw
    try:
        import glfw
        success(f"glfw: OK")
    except ImportError:
        warning("glfw: NOT INSTALLED (needed for GUI)")
        results["warnings"] += 1
    
    # ==================== Environment Variables ====================
    section("Environment Variables")
    
    env_vars = [
        ("CUDA_HOME", "/usr/local/cuda"),
        ("ROS_DOMAIN_ID", None),
        ("DISPLAY", None),
    ]
    
    for var, expected in env_vars:
        value = os.environ.get(var, None)
        if value:
            if expected and value != expected:
                warning(f"{var}: {value} (expected: {expected})")
            else:
                success(f"{var}: {value}")
        else:
            if var in ["ROS_DOMAIN_ID", "DISPLAY"]:
                warning(f"{var}: NOT SET")
                results["warnings"] += 1
            else:
                failure(f"{var}: NOT SET")
                results["failed"] += 1
    
    # ==================== Summary ====================
    section("Summary")
    
    total_checks = results["passed"] + results["failed"]
    
    if results["failed"] == 0:
        print(f"\n  {GREEN}{BOLD}All critical checks passed!{RESET}")
    else:
        print(f"\n  {RED}{BOLD}{results['failed']} critical issue(s) found{RESET}")
    
    if results["warnings"] > 0:
        print(f"  {YELLOW}{results['warnings']} warning(s){RESET}")
    
    print()
    
    return 0 if results["failed"] == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
