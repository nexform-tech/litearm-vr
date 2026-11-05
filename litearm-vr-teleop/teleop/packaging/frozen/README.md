# 把 teleop 服务端冻成自带解释器的可执行文件

目标机上**不需要 Python、不需要 conda、不需要 pip**：把 `teleop-server` 连同它旁边
那一整棵目录拷过去就能跑。模型不冻（142 MB 的 URDF/网格仍从仓库取），ROS2 不冻。

构建器：**PyInstaller 6.22（onedir）**，在 Docker 里跑（`ubuntu:22.04` + Python 3.10）。

## 构建

```bash
bash teleop/packaging/frozen/build.sh                 # 产物：<仓库根>/dist/teleop-frozen/
bash teleop/packaging/frozen/build.sh --rebuild-image # 改过 Dockerfile.builder 之后
```

单体构建 **约 25 秒**（PyInstaller 那一步 11–15 s；首次要在容器里 `pip install pyinstaller`，
再加 ~10 s）。产物 **300 MB**（不剥调试节是 427 MB）。中间产物在 `packaging-build/frozen/`，
两者都在 `.gitignore` 里。

### 为什么不用 Nuitka（上一版方案）

Nuitka 会把**每个纯 Python 模块**翻成 C 再 gcc 一遍：实测一次构建 1022 s，编了 666 个
`.o`，还停在 `scipy.stats`，一轮迭代 20 分钟 —— 打包器选型期间改一版试一次，这个成本
受不了。PyInstaller 直接收 `.pyc` + 编译好的扩展，同一套 `--collect-all` 白名单十几秒
出包。代价是产物比 Nuitka 大一点、启动时要解包 `base_library.zip`（毫秒级），对我们
这个「服务常驻」的场景无所谓。

**这个选择是暂定的**（2026-10-16 定）：先用起来再说，不是"PyInstaller 优于 Nuitka"的
定论。真要有更强理由（例如产物体积成了问题 —— 现在 300 MB，大头是 scipy/pinocchio 的
二进制与 142 MB 模型，跟打包器无关）再回头比一次，那时该量的是**产物体积与启动耗时**，
不是构建耗时。

## 产物长什么样

```
dist/teleop-frozen/
├── teleop-server                          # 入口（PyInstaller 引导器 + 内嵌 libpython3.10）
├── libpython3.10.so.1.0  base_library.zip  # 解释器与标准库
├── pinocchio/ coal/ eigenpy/ hppfcl/ …    # 扩展模块
├── cmeel.prefix/lib/*.so                  # pinocchio/coal/boost/urdfdom 的原生库
├── numpy.libs/ scipy.libs/ …              # OpenBLAS、gfortran
└── libpinocchio_default.so → cmeel.prefix/lib/…   # 顶层 27 个**相对符号链接**
```

* `--contents-directory .`：不让 PyInstaller 6 把东西塞进 `_internal/`，可执行文件与所有
  库名**在同一层**。
* 顶层那些是**相对符号链接**（PyInstaller 用来保住 `$ORIGIN` 相对布局，同时又让 soname
  在顶层能解析）。所以**整棵目录一起发**：`cp -a` / `rsync -a` / `tar` / dpkg 都没问题；
  但别用会丢符号链接的拷贝方式（FAT32 U 盘、`zip` 不带 `-y`），那会拷出一堆 0 字节文件。
  真要在无符号链接的文件系统上落盘，用 `cp -rL`（解引用，体积会涨到约 400 MB）。

## 调用契约（`launch/run_teleop_webxr.py --frozen <绝对路径>`）

冻结产物只认两个环境变量，跟不冻时一模一样：

| 变量 | 值 | 说明 |
| --- | --- | --- |
| `LITEARM_TELEOP_MODELS` | `<仓库根>/teleop/models` | URDF/网格从这儿加载（模型不进产物） |
| `LD_LIBRARY_PATH` | `teleop-server` 所在目录 | 调用方给的兜底；实测不给也能跑（PyInstaller 引导器会把自己的目录加进去） |

启动脚本已经替调用方把这两个变量设好了。手工验证可以直接跑：

```bash
cd /tmp/frozen-smoke   # 干净目录（整棵 dist/teleop-frozen 拷过来）
LITEARM_TELEOP_MODELS=<仓库根>/teleop/models \
  ./teleop-server --config <仓库根>/teleop/config/teleop.yaml \
  --source ws --no-real --ik pin --port 18766 --viewer rviz
```

## 构建脚本里都干了什么

`build.sh` → `docker run` → `build_in_container.sh`：装 PyInstaller → 打包 → 摆布局 →
收库 → 剥调试节 → 断言关键包 → 四道自检。

* **`--collect-all` 白名单**：`litearm_teleop pinocchio eigenpy coal hppfcl pink qpsolvers
  quadprog loop_rate_limiters zenoh websockets pybullet`；外加
  `--hidden-import real_arm_client probe`，排除 `rclpy` / `ros_joint_state`。
  `websockets` 整包收是必须的：它用 `websockets/imports.py` 的 `lazy_import` 在运行时
  才 import `websockets.asyncio`，静态分析看不见 —— 漏了的症状是「服务起得来、一到
  `websockets.serve` 就 ModuleNotFoundError」。
* **`collect_libs.py`**：兜底 + 复验。PyInstaller 自己的依赖分析（bindepend）这次已经
  把 cmeel 前缀（`libpinocchio_default/libcoal/libboost_*/liburdfdom_*`）、`numpy.libs`、
  `scipy.libs` 全收进来了（收库 0 个），脚本仍会拿 `ldd` 单独复验一遍「只用产物目录当
  `LD_LIBRARY_PATH`，还有没有 `not found`」—— 谁哪天收不全，构建会红而不是客户现场炸。
* **只删调试节**（`strip --strip-debug`）：pybullet 的 `.so` 光调试信息就 127 MB
  （136 MB → 9 MB），整包 427 MB → 300 MB。**跳过 `*.libs/`**：容器里的 binutils 2.38
  剥完 OpenBLAS 会让 `PT_LOAD` 不再页对齐，numpy 直接起不来（同一份文件在 Debian 的
  binutils 2.44 下剥完却没事）。也**不要**用 PyInstaller 的 `--strip`（连符号表一起剥，
  同样炸 OpenBLAS）。
* **关键包断言**：产物里必须有 `pinocchio_pywrap*.so`、`libpinocchio_default.so*`、
  `pybullet*.so`、`libpython3.10.so.1.0`，且顶层符号链接不许悬空/指向外部 ——
  缺一个就**构建失败，不出残缺产物**（照 litearm-studio `packaging/build.py` 的做法）。

## 四道自检（每次构建都跑，缺一不可）

| # | 判据 | 最近一次结果 |
| --- | --- | --- |
| ① | `--help` 退出码 0（不给 `LD_LIBRARY_PATH`） | 通过 |
| ② | `/tmp` 干净目录里无头起一次：`DIRECT` + `--ik pin`，URDF 从 `LITEARM_TELEOP_MODELS` 读 | ws、zenoh 两条数据源都起来（端口 LISTEN） |
| ③ | **pinocchio 真的被加载**：`/proc/<pid>/maps` 里有 `pinocchio_pywrap_default…so` 与 `cmeel.prefix/lib/*` | 命中 20 行（回退 pybullet = 失败） |
| ④ | `kill -TERM` 干净退出 | `rc=143` |

另外在**裸 `ubuntu:22.04`（没有 python3、没有 conda、没有 libX11）**里复验过 ①②③④，
全部通过 —— 这就是「目标机不需要 Python」的直接证据。`--config` 也是真读的：换一份
`/tmp` 下的配置（`tcp_offset` 0.18 → 0.25），日志里那行跟着变。

## 已知限制 / 坑

* **glibc 同代**：产物链接构建机（Ubuntu 22.04，glibc 2.35）的 glibc。目标机要 ≥ 同代；
  这台控制器/工控机如果是 Ubuntu 20.04，得把 `Dockerfile.builder` 的 `FROM` 换掉重打。
* **ROS2 不冻**：产物里没有 `rclpy`，服务端自己降级成「不发 `/joint_states`」。
* **默认 viewer 是 GUI**：`--viewer gui` 会开 pybullet 窗口，要求目标机有 libX11/libGL
  （桌面发行版都自带，产物**不带**这两个）。无头场景给 `--viewer rviz`（= pybullet
  `DIRECT`）。
* **`LITEARM_NO_VIEWER` 是个死变量**：代码里没有任何地方读它（只在 tests/docs 里出现），
  设了它**不会**关掉 GUI —— 关了窗口只能靠 `--viewer rviz` 或配置里的 `robot.viewer`。
* **服务端那几行就绪信息在重定向时是块缓冲的**（Python 默认行为，冻结与否都一样）：
  父进程如果把 stdout 接进管道/文件再等「`遥操仿真已就绪` / `数据源:`」出现，可能永远
  等不到。要么给它一个伪终端（`script -f -e -c …`），要么别等日志、直接探端口。
* **zenoh 是出厂数据源**，ws 那条路是给「头显直连」用的；两条都在自检里跑过。
