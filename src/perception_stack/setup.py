from setuptools import find_packages, setup

package_name = "perception_stack"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bottledsmoke",
    maintainer_email="bottledsmoke@todo.todo",
    description="Perception stack for drone navigation",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "depth_estimator = perception_stack.depth_estimator:main",
            "costmap_node = perception_stack.costmap_node:main",
            "vfh_planner = perception_stack.vfh_planner:main",
            "test_costmap = perception_stack.test_costmap:main",
        ],
    },
)
