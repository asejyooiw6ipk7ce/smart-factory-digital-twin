from setuptools import find_packages, setup

package_name = "factory_limo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="factory-project",
    maintainer_email="factory-project@example.com",
    description="Limo Pro 쪽 노드들 (WBS 2.x)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_detector = factory_limo.aruco_detector:main",
            "go_to_station_server = factory_limo.go_to_station_server:main",
            "world_pose_tracker = factory_limo.world_pose_tracker:main",
            "confirm_ui = factory_limo.confirm_ui:main",
        ],
    },
)
