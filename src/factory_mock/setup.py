from setuptools import find_packages, setup

package_name = "factory_mock"

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
    description="팩토리 프로젝트용 Limo/FR5 목업 액션 서버 (WBS 1.7)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_limo = factory_mock.mock_limo:main",
            "mock_fr5 = factory_mock.mock_fr5:main",
        ],
    },
)
