from setuptools import find_packages, setup
import os
from glob import glob

package_name = "fgo_transformer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}",
         ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "data"),
         glob("data/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yagiz Tansu",
    maintainer_email="yagiz@robot.local",
    description="Transformer trust-weight predictor — feature extraction node",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "feature_extractor    = fgo_transformer.feature_extractor_node:main",
            "transformer_inference = fgo_transformer.transformer_inference_node:main",
            "graph_traversal      = fgo_transformer.graph_traversal_node:main",
            "scenario_runner      = fgo_transformer.scenario_runner_node:main",
        ],
    },
)
