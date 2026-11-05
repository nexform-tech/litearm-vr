#!/usr/bin/env python3
"""STL 降采样脚本：为 pybullet 渲染生成低模副本（meshes_lowpoly/）。

按**统一缩减比例**降采样（而非固定目标面数），避免大部件被过度削减导致
形状不完整。之前的低模用了固定 ~3000 面，base_link/joint1（25 万面）
被砍到 1.2%，显示残缺；本脚本按比例保留每个部件 15% 面，形状完整。

依赖：trimesh + fast_simplification（仅生成时用，运行时不需要）
用法:
  python scripts/decimate_meshes.py --src models/litearm_description/meshes \
      --dst models/litearm_description/meshes_lowpoly [--reduction 0.85]
"""
import argparse
import os
import sys

import trimesh


def decimate_stl(src_path, dst_path, reduction):
    """读入 STL，按 reduction 比例（0~1，缩减比例）降采样，写出二进制 STL。"""
    mesh = trimesh.load(src_path, force="mesh")
    n_faces = len(mesh.faces)
    if n_faces == 0:
        print("  [跳过] %s 空网格" % os.path.basename(src_path))
        return
    # 小网格无需降采样（如 ee_link 仅 1242 面）
    if n_faces <= 2000:
        mesh.export(dst_path)
        print("  [原样] %s（%d 面，已足够小）" % (os.path.basename(src_path), n_faces))
        return
    from fast_simplification import simplify
    vertices, faces = simplify(mesh.vertices, mesh.faces,
                               target_reduction=reduction)
    out = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    out.export(dst_path)
    print("  [降采样] %s: %d → %d 面（保留 %.0f%%）"
          % (os.path.basename(src_path), n_faces, len(faces),
             (1 - reduction) * 100))


def main():
    ap = argparse.ArgumentParser(description="STL 降采样（pybullet 低模生成）")
    ap.add_argument("--src", required=True, help="源 meshes/ 目录")
    ap.add_argument("--dst", required=True, help="输出 meshes_lowpoly/ 目录")
    ap.add_argument("--reduction", type=float, default=0.85,
                    help="缩减比例 0~1（默认 0.85，即保留 15%%）")
    args = ap.parse_args()

    os.makedirs(args.dst, exist_ok=True)
    stl_files = sorted(f for f in os.listdir(args.src) if f.endswith(".STL"))
    if not stl_files:
        print("[错误] %s 下无 STL 文件" % args.src)
        sys.exit(1)

    print("降采样 %s → %s（缩减 %.0f%%）" % (args.src, args.dst, args.reduction * 100))
    for f in stl_files:
        decimate_stl(os.path.join(args.src, f), os.path.join(args.dst, f),
                     args.reduction)
    print("完成，共 %d 个文件" % len(stl_files))


if __name__ == "__main__":
    main()
