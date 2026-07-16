from setuptools import find_packages, setup

package_name = "fr3_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jau",
    maintainer_email="jau.gretler@gmail.com",
    description="fr3_teleop Python utilities and nodes",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "franka_wrench_extractor = fr3_teleop.franka_wrench_extractor:main",
            "teleop_dashboard = fr3_teleop.helpers.dashboard:main",
        ],
    },
)
