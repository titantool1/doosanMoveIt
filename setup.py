from setuptools import setup, find_packages
from pathlib import Path

package_name = "doosan_moveit_gui"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament index 등록(필수)
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # package.xml 설치(필수)
        ("share/" + package_name, ["package.xml"]),
        # launch 파일(선택)
        ("share/" + package_name + "/launch", list(map(str, Path("launch").glob("*.py")))),
        # 문서/이미지(선택)
        ("share/" + package_name + "/docs", list(map(str, Path("docs").glob("*.md")))),
        ("share/" + package_name + "/assets/screenshots", list(map(str, Path("assets/screenshots").glob("*")))),
        # yaml 리소스: 코드 옆에 둬도 되지만, 설치까지 깔끔하게 하려면 여기도 포함
        ("share/" + package_name + "/config", list(map(str, Path("doosan_moveit_gui").glob("*.yaml")))),
        # waypoints.yaml / waypoints2.yaml을 share/<pkg>/ 로 설치
        ("share/" + package_name, [
            "doosan_moveit_gui/waypoints.yaml",
            "doosan_moveit_gui/waypoints2.yaml",
            ]),
        ("share/" + package_name + "/rviz", list(map(str, Path("rviz").glob("*.rviz")))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="PyQt5 GUI to control Doosan robot via MoveIt2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            # ros2 run doosan_moveit_gui gui_app
            "gui_app = doosan_moveit_gui.gui_app:main",
        ],
    },
)
