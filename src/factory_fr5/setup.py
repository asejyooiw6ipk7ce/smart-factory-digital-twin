from setuptools import find_packages, setup

package_name = "factory_fr5"

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
        "FR5 로봇팔 제어 (WBS 3.3/3.4) — Fairino SDK 래퍼 + "
        "PICK/PLACE/HOME 로직 + Fr5Task 액션 서버"
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fr5_node = factory_fr5.fr5_node:main",
        ],
    },
)
