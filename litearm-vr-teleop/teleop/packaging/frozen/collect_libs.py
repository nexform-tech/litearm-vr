#!/usr/bin/env python3
"""把冻结产物需要、但 Nuitka 没收全的共享库收进产物目录（拍平）。

为什么要手工收
--------------
打包器只保证「Python 模块找得到」，不保证「模块链接的 .so 找得到」：

* pinocchio / eigenpy / coal / hppfcl 来自 cmeel 轮子。Python 包在
  `<site-packages>/cmeel.prefix/lib/python3.10/site-packages/` 下，而它们链接的
  `libpinocchio_default.so` / `libcoal.so` / `libboost_*.so` / `liburdfdom_*.so`
  在 `<site-packages>/cmeel.prefix/lib/` 下。这些扩展的 RUNPATH 是
  `$ORIGIN/../../../../lib` —— 只有保持 cmeel 的目录层级才找得到。
* numpy / scipy 同理：`numpy.libs/`、`scipy.libs/`（RPATH `$ORIGIN/../numpy.libs`）。

PyInstaller 的依赖分析（bindepend）能收掉一部分，但它的搜索结果随 RPATH 的写法而变，
不保证收全 —— 这个脚本是**兜底**：收完再验一遍，收不齐就退出码非 0。

而调用方（`launch/run_teleop_webxr.py --frozen`）给的契约是
「**所有共享库都跟可执行文件躺在同一个目录**，由 LD_LIBRARY_PATH 找」——
不套 shell 包装脚本，也不许有子目录。所以要把它们**拍平**收进产物目录。

做法
----
对产物目录里每个 ELF 跑 `ldd`：
  解析到 site-packages（cmeel 前缀 / *.libs）里的 → 拷到产物目录（同名），
  新拷进来的再跑一遍（求闭包）。
系统库（libc/libstdc++/libGL…）**不拷** —— glibc 由目标机的发行版提供，
Nuitka standalone 出来的东西本来就要求目标机与构建机同一代发行版。

最后再验一遍：只用产物目录当 LD_LIBRARY_PATH，看还有没有 `not found`。
凡是「在 site-packages 里解析得到、拍平后却找不到」的，判为失败（退出码 1）。

用法: python3 collect_libs.py <产物目录>
"""
import os
import shutil
import subprocess
import sys
import sysconfig


def vendor_dirs():
    """site-packages 根 + 里面所有会放 .so 的目录（cmeel 前缀、*.libs）。"""
    base = sysconfig.get_paths()["purelib"]
    dirs = [base]
    for sub in ("cmeel.prefix/lib", "cmeel.prefix/lib64"):
        path = os.path.join(base, sub)
        if os.path.isdir(path):
            dirs.append(path)
    for name in sorted(os.listdir(base)):
        path = os.path.join(base, name)
        if name.endswith(".libs") and os.path.isdir(path):
            dirs.append(path)
    return base, dirs


def elf_files(root):
    """root 下所有 ELF 文件（按魔数判断，不靠扩展名）。"""
    found = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in filenames:
            path = os.path.join(dirpath, name)
            try:
                with open(path, "rb") as fh:
                    if fh.read(4) == b"\x7fELF":
                        found.append(path)
            except OSError:
                pass
    return found


def ldd(path, ld_library_path):
    """`ldd` 的结果：[(NEEDED 名, 解析到的路径 或 'not found')]。"""
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = ld_library_path
    try:
        res = subprocess.run(["ldd", path], capture_output=True, text=True,
                             env=env)
    except OSError:
        return []
    libs = []
    for line in res.stdout.splitlines():
        if "=>" not in line:
            continue
        name, rest = line.split("=>", 1)
        libs.append((name.strip(), rest.strip().split(" (")[0].strip()))
    return libs


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    dist = os.path.abspath(sys.argv[1])
    vendor, search_dirs = vendor_dirs()
    search = ":".join(search_dirs)

    # 1) 闭包：把 site-packages 里的库拍平拷进产物目录
    copied = {}
    vendored_names = set()
    queue = elf_files(dist)
    n_seed = len(queue)
    while queue:
        path = queue.pop()
        for _name, target in ldd(path, search):
            if target in ("not found", ""):
                continue
            if not target.startswith(vendor + os.sep):
                continue
            basename = os.path.basename(target)
            vendored_names.add(basename)
            dest = os.path.join(dist, basename)
            if os.path.exists(dest) or basename in copied:
                continue
            shutil.copy2(target, dest)
            copied[basename] = target
            queue.append(dest)

    total = sum(os.path.getsize(p) for p in copied.values())
    print("    收库: %d 个 / %.1f MB（产物目录里原有 ELF %d 个）"
          % (len(copied), total / 1e6, n_seed))
    groups = {}
    for basename in copied:
        if basename.startswith("libboost"):
            key = "libboost_*"
        else:
            key = basename
        groups[key] = groups.get(key, 0) + 1
    if groups:
        print("    " + "、".join(sorted(groups)))

    # 2) 复验：只用产物目录当 LD_LIBRARY_PATH，还有没有找不到的
    unresolved = []
    for path in elf_files(dist):
        for name, target in ldd(path, dist):
            if target in ("not found", ""):
                unresolved.append((os.path.relpath(path, dist), name))

    hard = [x for x in unresolved if x[1] in vendored_names]
    soft = [x for x in unresolved if x[1] not in vendored_names]
    if soft:
        # 交给目标机的发行版（libGL/libX11/libgomp… 系统库）。列出来但不算失败。
        print("    [注意] 未随包携带（应由目标机发行版提供）: %s"
              % "、".join(sorted({n for _p, n in soft})))
    if hard:
        print("    [错误] 拍平后仍找不到（本该随包携带）:")
        for owner, name in sorted(set(hard)):
            print("        %s ← %s" % (name, owner))
        return 1
    print("    复验: 产物目录内所有 ELF 的依赖都能在产物目录里解析")
    return 0


if __name__ == "__main__":
    sys.exit(main())
