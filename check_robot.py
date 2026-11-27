#!/usr/bin/env python3
"""
check_robot.py - Diagnose why a robot "falls apart" in Gazebo Sim.

Checks a URDF, xacro or SDF file for the usual causes:
  1. Joints referencing missing links / broken kinematic tree (free-floating links)
  2. Mesh scale / units problems and large visual offsets
  3. Missing, tiny or physically invalid inertia
  4. Model spawned at/under the ground (SDF <pose>)
  5. High-poly mesh collisions
  6. Very high controller gains / large physics step size

Usage:
  python3 check_robot.py yantra.urdf
  python3 check_robot.py yantra.urdf.xacro --pkg-path ~/ros2_ws/src
  python3 check_robot.py model.sdf --world my_world.sdf
  python3 check_robot.py yantra.urdf --gz        # also run `gz sdf -p` conversion check

Only uses the Python standard library.
"""

import argparse
import math
import os
import re
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

# ----------------------------------------------------------------- thresholds
MIN_MASS = 1e-3           # kg   - below this solvers get unstable
MIN_INERTIA = 1e-8        # kg*m^2
MAX_VISUAL_OFFSET = 1.0   # m    - visual origin further than this from its link
MESH_MM_SUSPECT = 5.0     # m    - mesh bbox larger than this -> probably millimetres
MAX_COLLISION_TRIS = 5000 # triangles in a collision mesh
MAX_PID_GAIN = 1e4
MAX_STEP_SIZE = 0.005     # s

VALID_URDF_JOINTS = {"revolute", "continuous", "prismatic", "fixed", "floating", "planar"}


class Report:
    def __init__(self):
        self.items = []

    def add(self, level, category, msg):
        self.items.append((level, category, msg))

    def error(self, c, m): self.add("ERROR", c, m)
    def warn(self, c, m): self.add("WARN", c, m)
    def info(self, c, m): self.add("INFO", c, m)

    def print(self):
        colors = {"ERROR": "\033[91m", "WARN": "\033[93m", "INFO": "\033[94m"}
        reset = "\033[0m" if sys.stdout.isatty() else ""
        if not sys.stdout.isatty():
            colors = {k: "" for k in colors}
        cats = []
        for _, c, _ in self.items:
            if c not in cats:
                cats.append(c)
        for c in sorted(cats):
            print(f"\n== {c} ==")
            for lvl, cc, m in self.items:
                if cc == c:
                    print(f"  {colors[lvl]}[{lvl}]{reset} {m}")
        n_err = sum(1 for i in self.items if i[0] == "ERROR")
        n_warn = sum(1 for i in self.items if i[0] == "WARN")
        print(f"\nSummary: {n_err} error(s), {n_warn} warning(s)")
        return n_err


# ---------------------------------------------------------------- helpers
def floats(s, n=None, default=0.0):
    vals = [float(v) for v in s.split()] if s else []
    if n is not None:
        vals += [default] * (n - len(vals))
    return vals


def load_xml(path):
    if path.endswith(".xacro"):
        exe = shutil.which("xacro")
        if not exe:
            sys.exit("File is xacro but the `xacro` command was not found. "
                     "Run: xacro file.xacro > file.urdf  and pass the .urdf.")
        out = subprocess.run([exe, path], capture_output=True, text=True)
        if out.returncode != 0:
            sys.exit("xacro failed:\n" + out.stderr)
        return ET.fromstring(out.stdout)
    return ET.parse(path).getroot()


def resolve_uri(uri, base_dir, pkg_paths):
    """Resolve package://, model://, file:// and relative mesh paths."""
    if uri.startswith("file://"):
        return uri[7:]
    for prefix in ("package://", "model://"):
        if uri.startswith(prefix):
            rest = uri[len(prefix):]
            pkg, _, sub = rest.partition("/")
            search = list(pkg_paths)
            search += os.environ.get("GZ_SIM_RESOURCE_PATH", "").split(":")
            search += os.environ.get("ROS_PACKAGE_PATH", "").split(":")
            search.append(os.path.dirname(base_dir))
            for root in filter(None, search):
                cand = os.path.join(root, pkg, sub)
                if os.path.isfile(cand):
                    return cand
                # walk one level for workspace/src/<pkg>
                for dirpath, dirnames, _ in os.walk(root):
                    if os.path.basename(dirpath) == pkg:
                        cand = os.path.join(dirpath, sub)
                        if os.path.isfile(cand):
                            return cand
                    if dirpath.count(os.sep) - root.count(os.sep) > 3:
                        dirnames[:] = []
            return None
    cand = uri if os.path.isabs(uri) else os.path.join(base_dir, uri)
    return cand if os.path.isfile(cand) else None


def mesh_stats(path):
    """Return (bbox_size_xyz, triangle_count, unit_note) or None."""
    ext = os.path.splitext(path)[1].lower()
    try:
        if ext == ".stl":
            with open(path, "rb") as f:
                data = f.read()
            pts = []
            if data[:5].lower() == b"solid" and b"facet" in data[:1000]:
                for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data):
                    pts.append(tuple(float(v) for v in m.groups()))
                tris = len(pts) // 3
            else:
                tris = struct.unpack("<I", data[80:84])[0]
                for i in range(tris):
                    off = 84 + i * 50 + 12
                    for k in range(3):
                        pts.append(struct.unpack("<fff", data[off + 12 * k: off + 12 * k + 12]))
            return bbox(pts), tris, None
        if ext == ".obj":
            pts, tris = [], 0
            with open(path, errors="ignore") as f:
                for line in f:
                    if line.startswith("v "):
                        pts.append(tuple(float(v) for v in line.split()[1:4]))
                    elif line.startswith("f "):
                        tris += max(1, len(line.split()) - 3)
            return bbox(pts), tris, None
        if ext == ".dae":
            root = ET.parse(path).getroot()
            ns = {"c": root.tag.split("}")[0].strip("{")} if "}" in root.tag else {}
            unit = root.find(".//c:asset/c:unit", ns) if ns else root.find(".//asset/unit")
            meter = float(unit.get("meter", "1")) if unit is not None else 1.0
            pts, tris = [], 0
            for fa in root.iter():
                if fa.tag.endswith("float_array") and "position" in (fa.get("id") or "").lower():
                    v = floats(fa.text)
                    pts += [tuple(x * meter for x in v[i:i + 3]) for i in range(0, len(v) - 2, 3)]
                if fa.tag.endswith("triangles") or fa.tag.endswith("polylist"):
                    tris += int(fa.get("count", "0"))
            note = f"COLLADA unit meter={meter}"
            return bbox(pts), tris, note
    except Exception as e:  # noqa
        return None
    return None


def bbox(pts):
    if not pts:
        return None
    mins = [min(p[i] for p in pts) for i in range(3)]
    maxs = [max(p[i] for p in pts) for i in range(3)]
    return [maxs[i] - mins[i] for i in range(3)]


def check_inertia_tensor(rep, link, mass, I):
    ixx, ixy, ixz, iyy, iyz, izz = I
    cat = "3. Inertia"
    if mass <= 0:
        rep.error(cat, f"link '{link}': mass {mass} <= 0")
        return
    if mass < MIN_MASS:
        rep.warn(cat, f"link '{link}': very small mass {mass:g} kg (unstable)")
    diag = (ixx, iyy, izz)
    if any(d <= 0 for d in diag):
        rep.error(cat, f"link '{link}': non-positive principal inertia {diag}")
        return
    if any(d < MIN_INERTIA for d in diag):
        rep.warn(cat, f"link '{link}': tiny inertia {diag} (< {MIN_INERTIA:g})")
    # triangle inequality
    if ixx + iyy < izz or ixx + izz < iyy or iyy + izz < ixx:
        rep.error(cat, f"link '{link}': inertia violates triangle inequality "
                       f"(ixx={ixx:g}, iyy={iyy:g}, izz={izz:g}) - physically impossible")
    # positive definite (leading principal minors)
    m2 = ixx * iyy - ixy ** 2
    m3 = (ixx * (iyy * izz - iyz ** 2) - ixy * (ixy * izz - iyz * ixz)
          + ixz * (ixy * iyz - iyy * ixz))
    if m2 <= 0 or m3 <= 0:
        rep.error(cat, f"link '{link}': inertia matrix is not positive definite")
    # rough density sanity: inertia ~ m*r^2 -> r = sqrt(I/m)
    r = math.sqrt(max(diag) / mass)
    if r > 10:
        rep.warn(cat, f"link '{link}': inertia implies ~{r:.1f} m radius - "
                      f"maybe mm/g units or wrong mass")


# ---------------------------------------------------------------- parsing
def parse_urdf(root):
    links, joints, plugins = {}, [], []
    for l in root.findall("link"):
        name = l.get("name")
        d = {"inertial": None, "visuals": [], "collisions": []}
        inn = l.find("inertial")
        if inn is not None:
            m = inn.find("mass")
            i = inn.find("inertia")
            mass = float(m.get("value", "0")) if m is not None else 0.0
            I = [float(i.get(k, "0")) if i is not None else 0.0
                 for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")]
            d["inertial"] = (mass, I)
        for tag in ("visual", "collision"):
            for v in l.findall(tag):
                o = v.find("origin")
                xyz = floats(o.get("xyz") if o is not None else "", 3)
                mesh = v.find("geometry/mesh")
                entry = {"xyz": xyz, "mesh": None, "scale": [1, 1, 1]}
                if mesh is not None:
                    entry["mesh"] = mesh.get("filename")
                    entry["scale"] = floats(mesh.get("scale"), 3, 1.0) if mesh.get("scale") else [1, 1, 1]
                d[tag + "s"].append(entry)
        links[name] = d
    for j in root.findall("joint"):
        p, c = j.find("parent"), j.find("child")
        joints.append({"name": j.get("name"), "type": j.get("type"),
                       "parent": p.get("link") if p is not None else None,
                       "child": c.get("link") if c is not None else None})
    for p in root.iter("plugin"):
        plugins.append(p)
    return links, joints, plugins, None


def parse_sdf(root):
    model = root.find(".//model")
    if model is None:
        sys.exit("No <model> found in SDF.")
    links, joints = {}, []
    for l in model.iter("link"):
        name = l.get("name")
        d = {"inertial": None, "visuals": [], "collisions": []}
        inn = l.find("inertial")
        if inn is not None:
            m = inn.find("mass")
            mass = float(m.text) if m is not None else 1.0  # SDF default mass is 1
            i = inn.find("inertia")
            def g(k, default):
                e = i.find(k) if i is not None else None
                return float(e.text) if e is not None else default
            I = [g("ixx", 1), g("ixy", 0), g("ixz", 0), g("iyy", 1), g("iyz", 0), g("izz", 1)]
            d["inertial"] = (mass, I)
        for tag in ("visual", "collision"):
            for v in l.findall(tag):
                pose = v.find("pose")
                xyz = floats(pose.text if pose is not None else "", 6)[:3]
                mesh = v.find("geometry/mesh")
                entry = {"xyz": xyz, "mesh": None, "scale": [1, 1, 1]}
                if mesh is not None:
                    uri = mesh.find("uri")
                    sc = mesh.find("scale")
                    entry["mesh"] = uri.text.strip() if uri is not None else None
                    entry["scale"] = floats(sc.text, 3, 1.0) if sc is not None else [1, 1, 1]
                d[tag + "s"].append(entry)
        links[name] = d
    for j in model.iter("joint"):
        p, c = j.find("parent"), j.find("child")
        joints.append({"name": j.get("name"), "type": j.get("type"),
                       "parent": p.text.strip() if p is not None else None,
                       "child": c.text.strip() if c is not None else None})
    pose = model.find("pose")
    model_pose = floats(pose.text, 6) if pose is not None else None
    return links, joints, list(model.iter("plugin")), model_pose


# ---------------------------------------------------------------- checks
def check_tree(rep, links, joints, is_urdf):
    cat = "1. Joints / kinematic tree"
    names = set(links)
    rep.info(cat, f"{len(links)} links, {len(joints)} joints")
    seen = set()
    parent_of = {}
    for j in joints:
        if j["name"] in seen:
            rep.error(cat, f"duplicate joint name '{j['name']}'")
        seen.add(j["name"])
        if is_urdf and j["type"] not in VALID_URDF_JOINTS:
            rep.error(cat, f"joint '{j['name']}' has invalid type '{j['type']}'")
        for side in ("parent", "child"):
            ln = j[side]
            if ln is None:
                rep.error(cat, f"joint '{j['name']}' has no {side}")
            elif ln not in names and ln != "world":
                rep.error(cat, f"joint '{j['name']}' {side} link '{ln}' does not exist "
                               f"-> joint dropped, link will fall freely")
        if j["child"] in parent_of:
            rep.error(cat, f"link '{j['child']}' has two parent joints "
                           f"('{parent_of[j['child']]}' and '{j['name']}')")
        parent_of[j["child"]] = j["name"]

    children = {j["child"] for j in joints}
    roots = [n for n in links if n not in children]
    if len(roots) > 1:
        rep.error(cat, f"{len(roots)} root links (not attached to anything): {roots} "
                       f"-> every extra root is a separate free body")
    elif roots:
        rep.info(cat, f"root link: '{roots[0]}'")

    # massless links in URDF get removed by the URDF->SDF converter
    if is_urdf:
        for j in joints:
            c = j["child"]
            if c in links and j["type"] != "fixed":
                inn = links[c]["inertial"]
                if inn is None or inn[0] <= 0:
                    rep.error(cat, f"link '{c}' (child of non-fixed joint '{j['name']}') "
                                   f"has no mass -> converter drops it and its joint")


def check_inertia(rep, links, is_urdf):
    cat = "3. Inertia"
    for name, d in links.items():
        if d["inertial"] is None:
            if is_urdf:
                rep.warn(cat, f"link '{name}': no <inertial> (massless; dropped or "
                              f"lumped by the converter)")
            else:
                rep.warn(cat, f"link '{name}': no <inertial> - SDF default "
                              f"(1 kg, I=1) is used, rarely realistic")
            continue
        mass, I = d["inertial"]
        check_inertia_tensor(rep, name, mass, I)


def check_meshes(rep, links, base_dir, pkg_paths):
    cat2 = "2. Meshes / visual offsets"
    cat5 = "5. Collision geometry"
    n_mesh_col = 0
    cache = {}
    for name, d in links.items():
        for v in d["visuals"]:
            off = math.sqrt(sum(x * x for x in v["xyz"]))
            if off > MAX_VISUAL_OFFSET:
                rep.warn(cat2, f"link '{name}': visual origin is {off:.2f} m from the link "
                               f"frame - parts will look detached")
        for kind in ("visuals", "collisions"):
            for v in d[kind]:
                if not v["mesh"]:
                    continue
                if kind == "collisions":
                    n_mesh_col += 1
                path = resolve_uri(v["mesh"], base_dir, pkg_paths)
                if path is None:
                    rep.warn(cat2, f"link '{name}': cannot find mesh '{v['mesh']}' "
                                   f"(use --pkg-path); in Gazebo it may be invisible")
                    continue
                if path not in cache:
                    cache[path] = mesh_stats(path)
                st = cache[path]
                if st is None:
                    continue
                size, tris, note = st
                if size:
                    scaled = [size[i] * v["scale"][i] for i in range(3)]
                    big = max(scaled)
                    if big > MESH_MM_SUSPECT:
                        rep.error(cat2, f"link '{name}' {kind[:-1]} '{os.path.basename(path)}' "
                                        f"is {big:.1f} m across after scale {v['scale']} "
                                        f"-> probably in millimetres, use scale 0.001")
                    elif 0 < big < 0.001:
                        rep.warn(cat2, f"link '{name}' {kind[:-1]} '{os.path.basename(path)}' "
                                       f"is only {big*1000:.3f} mm across - double-scaled?")
                if kind == "collisions" and tris > MAX_COLLISION_TRIS:
                    rep.warn(cat5, f"link '{name}': collision mesh '{os.path.basename(path)}' "
                                   f"has {tris} triangles - use a primitive or simplified hull")
    if n_mesh_col:
        rep.info(cat5, f"{n_mesh_col} collision(s) use meshes; primitives are more stable")
    else:
        rep.info(cat5, "no mesh collisions")


def check_spawn(rep, model_pose):
    cat = "4. Spawn pose"
    if model_pose is None:
        rep.info(cat, "no model <pose> in file; make sure your spawn z is above the ground "
                      "(e.g. ros_gz_sim create -z 0.3)")
    elif model_pose[2] <= 0.0:
        rep.warn(cat, f"model pose z = {model_pose[2]} -> robot may start inside the "
                      f"ground and get launched; raise it")


def check_plugins(rep, plugins, world_path):
    cat = "6. Controllers / physics"
    for p in plugins:
        pname = p.get("name") or p.get("filename")
        for e in p.iter():
            tag = e.tag.lower()
            if tag in ("p_gain", "p", "kp", "i_gain", "d_gain") and e.text:
                try:
                    val = float(e.text)
                except ValueError:
                    continue
                if abs(val) > MAX_PID_GAIN:
                    rep.warn(cat, f"plugin '{pname}': {e.tag}={val:g} is very high - "
                                  f"can tear the robot apart; try disabling the plugin")
    if world_path:
        w = ET.parse(world_path).getroot()
        for ph in w.iter("physics"):
            st = ph.find("max_step_size")
            if st is not None and float(st.text) > MAX_STEP_SIZE:
                rep.warn(cat, f"world max_step_size={st.text} s is large; try 0.001")
            else:
                rep.info(cat, f"world max_step_size="
                              f"{st.text if st is not None else 'default (0.001)'}")
    else:
        rep.info(cat, "pass --world to also check physics step size")


def run_gz_check(rep, path):
    cat = "0. gz sdf conversion"
    exe = shutil.which("gz")
    if not exe:
        rep.warn(cat, "`gz` command not found; skipped conversion check")
        return
    out = subprocess.run([exe, "sdf", "-p", path], capture_output=True, text=True)
    msgs = [l for l in out.stderr.splitlines() if l.strip()]
    if out.returncode != 0:
        rep.error(cat, "gz sdf -p failed")
    for m in msgs[:30]:
        (rep.error if "Error" in m else rep.warn)(cat, m.strip())
    if out.returncode == 0:
        try:
            conv = ET.fromstring(out.stdout)
            mdl = conv.find(".//model")
            n_l = len(list(mdl.iter("link")))
            n_j = len(list(mdl.iter("joint")))
            rep.info(cat, f"after conversion: {n_l} links, {n_j} joints "
                          f"(compare with the counts in section 1)")
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="URDF, .xacro or SDF of the robot")
    ap.add_argument("--pkg-path", action="append", default=[],
                    help="directory to search for package:// or model:// meshes (repeatable)")
    ap.add_argument("--world", help="world SDF to check physics settings")
    ap.add_argument("--gz", action="store_true", help="also run `gz sdf -p` on the file")
    args = ap.parse_args()

    root = load_xml(args.file)
    is_urdf = root.tag == "robot"
    if not is_urdf and root.tag != "sdf":
        sys.exit(f"Unknown root tag <{root.tag}>; expected <robot> or <sdf>.")
    links, joints, plugins, model_pose = (parse_urdf if is_urdf else parse_sdf)(root)

    rep = Report()
    base = os.path.dirname(os.path.abspath(args.file))
    pkg_paths = [os.path.expanduser(p) for p in args.pkg_path]

    print(f"Checking {args.file} ({'URDF' if is_urdf else 'SDF'})")
    if args.gz and is_urdf:
        run_gz_check(rep, args.file)
    check_tree(rep, links, joints, is_urdf)
    check_meshes(rep, links, base, pkg_paths)
    check_inertia(rep, links, is_urdf)
    check_spawn(rep, model_pose)
    check_plugins(rep, plugins, args.world)

    n_err = rep.print()
    print("\nIn Gazebo, also try: right-click the model -> View -> "
          "Joints / Collisions / Inertia / Frames")
    sys.exit(1 if n_err else 0)


if __name__ == "__main__":
    main()
