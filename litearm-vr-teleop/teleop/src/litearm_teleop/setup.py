from setuptools import setup

setup(
    name="litearm-teleop",
    version="0.2.0",
    packages=["litearm_teleop"],
    package_dir={"litearm_teleop": "."},
    # URDF/meshes 已移到项目根目录（teleop/models/litearm_description/、
    # teleop/models/dual_litearm_column_description/），
    # 不再打包进包内（teleop_sim.py 用绝对路径直接加载）。
)
