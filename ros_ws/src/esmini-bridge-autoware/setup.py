from setuptools import find_packages, setup

package_name = "esmini_bridge_autoware"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="LolaInTa Chi",
    maintainer_email="lolainta@gmail.com",
    description="ESmini bridge for Autoware",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["esmini_bridge_autoware = src.esmini_bridge_autoware:main"],
    },
)
