from setuptools import find_packages, setup

package_name = "twist_converter"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dmitrytorov",
    maintainer_email="dmitrytorov@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "twist_to_twist_stamped = twist_converter.twist_to_twist_stamped:main",
        ],
    },
)
