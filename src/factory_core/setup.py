from setuptools import find_packages, setup

package_name = "factory_core"

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
    description=(
        "팩토리 프로젝트의 ROS-독립 핵심 로직(Job/JobRunner/InventoryDB) "
        "+ orchestrator_node.py (WBS 1.2~1.6, 1.8)"
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "orchestrator_node = factory_core.orchestrator_node:main",
        ],
    },
)
