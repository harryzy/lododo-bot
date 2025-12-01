from setuptools import find_packages, setup
from glob import glob

package_name = "bot_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # launch setup
        ("share/" + package_name + "/launch", glob("launch/py/*_launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf") + glob("urdf/*.xacro") + glob("urdf/*.rviz")),
        # config setup
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # meshes setup
        ("share/" + package_name + "/urdf/meshes", glob("urdf/meshes/*.stl")),
        ("share/" + package_name + "/urdf/meshes", glob("urdf/meshes/*.dae")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lododo",
    maintainer_email="contect@lododo.org",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    # entry_points={
    #     'console_scripts': [
    #         'bot_description = bot_description.bot_description:main'
    #     ],
    # },
)
