from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

with open("requirements.txt", "r", encoding="utf-8") as fh:
    requirements = [line.strip() for line in fh if line.strip() and not line.startswith("#")]

setup(
    name="vla-robotics",
    version="1.0.0",
    author="VLA Development Team",
    author_email="vla-team@example.com",
    description="Vision-Language-Action system for humanoid robotics",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/your-org/vla-system",
    packages=find_packages(where="src", include=["vla", "vla.*"]),
    package_dir={"": "src"},
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
    ],
    python_requires=">=3.8",
    install_requires=requirements,
    entry_points={
        'console_scripts': [
            'vla-voice-node=src.ros2_nodes.voice_command_node:main',
            'vla-planner-node=src.ros2_nodes.cognitive_planner_node:main',
            'vla-controller-node=src.ros2_nodes.humanoid_controller_node:main',
        ],
    },
)