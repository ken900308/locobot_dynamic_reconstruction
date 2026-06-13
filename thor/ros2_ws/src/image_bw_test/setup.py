from setuptools import setup

package_name = "image_bw_test"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Compressed image bandwidth comparison node for multi-camera fusion tests",
    license="MIT",
    entry_points={
        "console_scripts": [
            "compressed_image_bw_test_node = image_bw_test.compressed_image_bw_test_node:main",
        ],
    },
)
