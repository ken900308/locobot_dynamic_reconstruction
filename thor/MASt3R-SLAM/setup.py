import os
from setuptools import setup

os.environ["TORCH_CUDA_ARCH_LIST"] = "11.0"

import torch
from torch.utils.cpp_extension import BuildExtension, CppExtension, CUDAExtension

ROOT = os.path.dirname(os.path.abspath(__file__))
has_cuda = torch.cuda.is_available()

include_dirs = [
    os.path.join(ROOT, "mast3r_slam/backend/include"),
    os.path.join(ROOT, "thirdparty/eigen"),
]

sources = [
    "mast3r_slam/backend/src/gn.cpp",
]

extra_compile_args = {
    "cxx": ["-O3"],
}

ext_modules = []

if has_cuda:
    sources.append("mast3r_slam/backend/src/gn_kernels.cu")
    sources.append("mast3r_slam/backend/src/matching_kernels.cu")
    extra_compile_args["nvcc"] = [
        "-O3",
        "-gencode=arch=compute_110,code=sm_110",
    ]

    ext_modules = [
        CUDAExtension(
            "mast3r_slam_backends",
            include_dirs=include_dirs,
            sources=sources,
            extra_compile_args=extra_compile_args,
        )
    ]
else:
    print("CUDA not found, cannot compile backend!")
    ext_modules = [
        CppExtension(
            "mast3r_slam_backends",
            include_dirs=include_dirs,
            sources=sources,
            extra_compile_args=extra_compile_args,
        )
    ]

setup(
    ext_modules=ext_modules,
    cmdclass={"build_ext": BuildExtension},
)

