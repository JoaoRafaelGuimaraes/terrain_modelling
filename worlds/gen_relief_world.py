#!/usr/bin/env python3
"""Versao com relevo de um mundo plano do LIMO: troca o grass_plane pelo chao do Cerrado
(so o terreno, sem troncos, arbustos e galhos do modelo original) e assenta cada arvore no chao.

  python3 gen_relief_world.py forest_clustered_10min_limo.sdf forest_clustered_10min_relief_limo.sdf

O z de cada arvore e a MENOR altura do terreno sob o tronco (centro + circulo de raio TRUNK_R),
para nao sobrar fresta no lado de baixo da encosta. O tronco fica de pe (nao segue a inclinacao).
"""

import re
import struct
import sys

import numpy as np

STL = "/root/limo_ws/src/limo_cerrado_sim/models/cerrado_32x32_15_arvores/meshes/terrain_collision.stl"
TRUNK_R = 0.12  # [m] maior raio de tronco dos modelos benchmark_* (DAP 0,23 m)

HEADER = """<!--
    {name}: as arvores de {source}, no chao do Cerrado 32 x 32 m
    (relevo de 0 a 2,7 m em x/y [-16, 16]; nao ha chao fora do quadrado). Gerado por gen_relief_world.py.
    Unicas colisoes: o terreno e as arvores. O nome "cerrado" e obrigatorio: o limo_cerrado.launch.py
    faz o spawn em -world cerrado; o z:=auto do launch le o mesmo terrain_collision.stl usado aqui.
    Modelos: octomap_gazebo_forest/models e models/forest-gen-models no GZ_SIM_RESOURCE_PATH (feito na
    session.yml); model://cerrado_32x32_15_arvores vem do share do limo_cerrado_sim (feito pelo launch).
  -->"""

TERRAIN = """<model name="terrain">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>model://cerrado_32x32_15_arvores/meshes/terrain.dae</uri></mesh></geometry>
          <cast_shadows>true</cast_shadows>
        </visual>
        <collision name="collision">
          <geometry><mesh><uri>model://cerrado_32x32_15_arvores/meshes/terrain_collision.stl</uri></mesh></geometry>
          <surface><friction><ode><mu>0.85</mu><mu2>0.85</mu2></ode></friction></surface>
        </collision>
      </link>
    </model>"""


def load_triangles(path):
    with open(path, "rb") as f:
        f.seek(80)
        n = struct.unpack("<I", f.read(4))[0]
        rec = np.frombuffer(f.read(n * 50), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3, 3).astype(np.float64)


def terrain_z(tri, x, y):
    """Altura do terreno em (x, y), ou None fora da malha (mesma conta do limo_cerrado.launch.py)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    v0, v1 = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2]
    v2 = np.array([x, y]) - a[:, :2]
    den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
    ok = np.abs(den) > 1e-12
    den = np.where(ok, den, 1.0)
    u = (v2[:, 0] * v1[:, 1] - v1[:, 0] * v2[:, 1]) / den
    w = (v0[:, 0] * v2[:, 1] - v2[:, 0] * v0[:, 1]) / den
    idx = np.flatnonzero(ok & (u >= -1e-9) & (w >= -1e-9) & (u + w <= 1 + 1e-9))
    if idx.size == 0:
        return None
    z = a[idx, 2] + u[idx] * (b[idx, 2] - a[idx, 2]) + w[idx] * (c[idx, 2] - a[idx, 2])
    return float(z.max())


def main():
    src_path, dst_path = sys.argv[1], sys.argv[2]
    text = open(src_path).read()
    tri = load_triangles(STL)
    ring = [(TRUNK_R * np.cos(t), TRUNK_R * np.sin(t)) for t in np.linspace(0, 2 * np.pi, 8, endpoint=False)]

    def seat(match):
        name, x, y, rest = match.group(1), float(match.group(2)), float(match.group(3)), match.group(4)
        zs = [terrain_z(tri, x + dx, y + dy) for dx, dy in [(0.0, 0.0)] + ring]
        zs = [z for z in zs if z is not None]
        if not zs:
            raise RuntimeError(f"{name} em ({x}, {y}) esta fora do terreno")
        print(f"{name}: ({x:7.3f}, {y:7.3f})  z = {min(zs):.3f} m  (desnivel sob o tronco {max(zs) - min(zs):.3f} m)")
        return f"<name>{name}</name><pose>{x:.3f} {y:.3f} {min(zs):.3f} {rest}</pose>"

    text, n_trees = re.subn(r"<name>([^<]+)</name><pose>([-\d.]+) ([-\d.]+) 0 ([^<]*)</pose>", seat, text)
    text, n_ground = re.subn(r"<include>\s*<uri>model://mrs_gazebo_common_resources/models/grass_plane</uri>.*?</include>",
                             TERRAIN, text, flags=re.S)
    name = dst_path.rsplit("/", 1)[-1]
    text = re.sub(r"<!--.*?-->", HEADER.format(name=name, source=src_path.rsplit("/", 1)[-1]), text, count=1, flags=re.S)
    if n_ground != 1 or n_trees == 0:
        raise RuntimeError(f"esperava 1 grass_plane e arvores; achei {n_ground} e {n_trees}")
    open(dst_path, "w").write(text)
    print(f"{n_trees} arvores -> {dst_path}")


if __name__ == "__main__":
    main()
