#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_pilot_viewer.py  —  把大 STL 抽稀 + 航点渲染成【单文件、完全离线】的 HTML 3D 查看器。

用途
----
给飞手一个浏览器里就能打开(Chrome/Edge, 手机也可)的 viewer.html, 直观查看喷涂路径与建筑
的对应关系, 解决 RViz 不直观、不易分享的问题。输出 HTML 内联了 three.js 与全部几何数据,
断网也能打开。查看器坐标系与航点 CSV 完全一致 (mission0 相对系: x=前指向建筑, y=左, z=离地高)。

依赖
----
  pip install open3d numpy          # 首选 (open3d 负责 STL 抽稀)
  # 若 open3d 读取超大 STL 失败, 脚本会尝试用 trimesh 读入后再交给 open3d 简化:
  pip install trimesh
  (本机请用 python3.8, 默认 python3 可能缺 numpy/open3d)

three.js: 优先读取本目录 vendor/three.min.js 与 vendor/OrbitControls.js(离线); 缺失时从 CDN 下载并缓存。

命令行
------
  python3.8 make_pilot_viewer.py \
      --stl models/wall1.stl \
      --csv /tmp/ship_painter_export/waypoints_full.csv \
      --pilot-csv /tmp/ship_painter_export/waypoints_pilot.csv \
      --meta /tmp/ship_painter_export/export_meta.json \
      --target-faces 150000 \
      --out viewer.html

STL 抽稀流水线
--------------
  1) open3d 读入, 打印原始顶点/面数与峰值内存;
  2) 体素聚类 simplify_vertex_clustering (快且省内存, 体素取 min_axis/400, 边界保真, AABB 偏差<<1%);
  3) 二次二次误差简化 simplify_quadric_decimation(target-faces);
  4) 清理退化/重复三角形;
  5) 校验: 抽稀前后 AABB 各轴尺寸偏差 < 1%, 面数落在 [0.5,1.3]*target; 不满足则用更细体素重试;
  6) 抽稀结果另存 <out同目录>/decimated_preview.stl 供人工肉眼检查(仅作视觉参照, 不参与规划)。

STL 到相对坐标系的摆放 (与 ship_painter 工程一致, bbox 用抽稀【前】原始网格)
  t_m.x = mission_forward_distance - bbox_min.x
  t_m.y = -(bbox_min.y + bbox_max.y)/2
  t_m.z = (-mission_fcu_ground_height + mission_bottom_clearance) - bbox_min.z
  p_view = p_stl_local + t_m ; 再统一 z += mission_fcu_ground_height (对齐 CSV 的离地高)
注意: 规划模型(boat2:wall1.stl)与展示 STL 若不同文件会错位, 请传入与规划一致的 STL。
"""

import argparse
import base64
import csv
import colorsys
import json
import math
import os
import resource
import sys
import time

import numpy as np

VENDOR_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vendor")
THREE_URL = "https://unpkg.com/three@0.128.0/build/three.min.js"
ORBIT_URL = "https://unpkg.com/three@0.128.0/examples/js/controls/OrbitControls.js"


def peak_mb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


# ---------------------------------------------------------------------------
# three.js 获取(本地优先, 否则下载缓存)
# ---------------------------------------------------------------------------
def load_vendor_js(fname, url):
    path = os.path.join(VENDOR_DIR, fname)
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return f.read()
    print("  vendor/%s 不存在, 尝试从 CDN 下载: %s" % (fname, url))
    try:
        import urllib.request
        os.makedirs(VENDOR_DIR, exist_ok=True)
        with urllib.request.urlopen(url, timeout=30) as r:
            data = r.read().decode("utf-8")
        with open(path, "w", encoding="utf-8") as f:
            f.write(data)
        print("  已缓存到 %s (%d bytes)" % (path, len(data)))
        return data
    except Exception as e:
        raise RuntimeError("无法获取 %s: %s\n请手动下载 %s 放到 %s/"
                           % (fname, e, url, VENDOR_DIR))


# ---------------------------------------------------------------------------
# STL 抽稀
# ---------------------------------------------------------------------------
def decimate_mesh(stl_path, target_faces):
    """返回 (verts_local Nx3 float64, tris Mx3 int32, bbox_min, bbox_max, orig_faces, mq_o3d)."""
    import open3d as o3d
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

    t0 = time.time()
    print("[decimate] reading %s ..." % stl_path)
    mesh = o3d.io.read_triangle_mesh(stl_path)
    if len(mesh.triangles) == 0:
        # open3d 读失败 -> 尝试 trimesh
        print("[decimate] open3d 读取为空, 尝试 trimesh ...")
        import trimesh
        tm = trimesh.load(stl_path, force="mesh")
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(np.asarray(tm.vertices, dtype=np.float64)),
            o3d.utility.Vector3iVector(np.asarray(tm.faces, dtype=np.int32)))
    orig_faces = len(mesh.triangles)
    aabb0 = mesh.get_axis_aligned_bounding_box()
    bbox_min = np.asarray(aabb0.min_bound, dtype=np.float64)
    bbox_max = np.asarray(aabb0.max_bound, dtype=np.float64)
    ext0 = bbox_max - bbox_min
    print("[decimate] read %.1fs  V=%d F=%d  AABB ext=%s  peak=%.0fMB"
          % (time.time() - t0, len(mesh.vertices), orig_faces,
             np.round(ext0, 3), peak_mb()))

    min_axis = float(np.min(ext0[ext0 > 1e-6])) if np.any(ext0 > 1e-6) else 1.0

    def run(voxel):
        t1 = time.time()
        mc = mesh.simplify_vertex_clustering(
            voxel_size=voxel,
            contraction=o3d.geometry.SimplificationContraction.Average)
        print("[decimate]   cluster voxel=%.4f -> F=%d (%.1fs, peak=%.0fMB)"
              % (voxel, len(mc.triangles), time.time() - t1, peak_mb()))
        t2 = time.time()
        mq = mc.simplify_quadric_decimation(target_number_of_triangles=int(target_faces))
        mq.remove_degenerate_triangles()
        mq.remove_duplicated_vertices()
        mq.remove_duplicated_triangles()
        mq.remove_unreferenced_vertices()
        print("[decimate]   quadric -> F=%d (%.1fs, peak=%.0fMB)"
              % (len(mq.triangles), time.time() - t2, peak_mb()))
        return mq

    voxel = max(min_axis / 400.0, 0.05)
    mq = run(voxel)
    # 校验 AABB, 不合格则用更细体素重试(更细=边界更保真)
    for _ in range(2):
        extq = np.asarray(mq.get_axis_aligned_bounding_box().get_extent())
        dev = np.abs(extq - ext0) / np.maximum(ext0, 1e-9) * 100.0
        if np.all(dev < 1.0):
            break
        print("[decimate]   AABB dev%%=%s >=1%%, 更细体素重试" % np.round(dev, 3))
        voxel *= 0.5
        mq = run(voxel)

    extq = np.asarray(mq.get_axis_aligned_bounding_box().get_extent())
    dev = np.abs(extq - ext0) / np.maximum(ext0, 1e-9) * 100.0
    nf = len(mq.triangles)
    face_ok = 0.5 * target_faces <= nf <= 1.3 * target_faces
    aabb_ok = bool(np.all(dev < 1.0))
    print("[decimate] DONE total=%.1fs peak=%.0fMB  faces=%d(ok=%s)  AABB dev%%=%s(ok=%s)"
          % (time.time() - t0, peak_mb(), nf, face_ok, np.round(dev, 4), aabb_ok))
    if not face_ok:
        print("[decimate] WARNING: 面数不在 [0.5,1.3]*target 区间")
    if not aabb_ok:
        print("[decimate] WARNING: AABB 偏差 >=1%, 请检查(仍可用于视觉参照)")

    verts = np.asarray(mq.vertices, dtype=np.float64)
    tris = np.asarray(mq.triangles, dtype=np.int32)
    return verts, tris, bbox_min, bbox_max, orig_faces, mq


def place_vertices(verts, bbox_min, bbox_max, meta):
    """局部系顶点 -> mission0 相对(离地)视图系, 与 CSV 一致。"""
    fwd = float(meta["mission_forward_distance"])
    hg = float(meta["mission_fcu_ground_height"])
    clr = float(meta["mission_bottom_clearance"])
    tmx = fwd - bbox_min[0]
    tmy = -(bbox_min[1] + bbox_max[1]) / 2.0
    tmz = (-hg + clr) - bbox_min[2]
    out = verts.copy()
    out[:, 0] += tmx
    out[:, 1] += tmy
    out[:, 2] += tmz + hg   # 再统一加离地高
    return out, (tmx, tmy, tmz)


# ---------------------------------------------------------------------------
# 航点 CSV
# ---------------------------------------------------------------------------
def read_layers(csv_path):
    """返回 (layers, total_pts, total_dist). layers=[{'idx':int,'z':float,'pos':np.Nx3}]"""
    from collections import OrderedDict
    buckets = OrderedDict()
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            li = int(float(r["layer_index"]))
            buckets.setdefault(li, []).append(
                (float(r["x"]), float(r["y"]), float(r["z"])))
    layers = []
    total_pts = 0
    total_dist = 0.0
    prev = None
    for li, pts in buckets.items():
        arr = np.asarray(pts, dtype=np.float32)
        # 层内 + 层间累计里程(按文件顺序=飞行顺序)
        seq = arr
        if prev is not None and len(seq) > 0:
            total_dist += float(np.linalg.norm(seq[0] - prev))
        if len(seq) > 1:
            total_dist += float(np.sum(np.linalg.norm(np.diff(seq, axis=0), axis=1)))
        if len(seq) > 0:
            prev = seq[-1]
        layers.append({"idx": li, "z": float(np.mean(arr[:, 2])), "pos": arr})
        total_pts += len(arr)
    return layers, total_pts, total_dist


def read_pilot(csv_path):
    """返回 dict: pos(np.Nx3 float32), labels=[[layer,wp,x,y,z],...]"""
    pos = []
    labels = []
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            x, y, z = float(r["x"]), float(r["y"]), float(r["z"])
            pos.append((x, y, z))
            labels.append([int(float(r["layer_index"])), int(float(r["wp_index"])),
                           round(x, 2), round(y, 2), round(z, 2)])
    return {"pos": np.asarray(pos, dtype=np.float32), "labels": labels}


def b64_f32(arr):
    return base64.b64encode(np.asarray(arr, dtype="<f4").tobytes()).decode("ascii")


def b64_u32(arr):
    return base64.b64encode(np.asarray(arr, dtype="<u4").tobytes()).decode("ascii")


# ---------------------------------------------------------------------------
# HTML 生成
# ---------------------------------------------------------------------------
def build_html(three_js, orbit_js, data):
    data_json = json.dumps(data, ensure_ascii=False)
    return (HTML_HEAD
            + "<script>\n" + three_js + "\n</script>\n"
            + "<script>\n" + orbit_js + "\n</script>\n"
            + "<script>\nwindow.__DATA__ = " + data_json + ";\n</script>\n"
            + "<script>\n" + APP_JS + "\n</script>\n"
            + HTML_TAIL)


HTML_HEAD = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>喷涂路径查看器 (飞手离线版)</title>
<style>
  html,body{margin:0;padding:0;height:100%;background:#0f1117;color:#e8e8ee;
    font-family:'Microsoft YaHei','PingFang SC',sans-serif;overflow:hidden;}
  #c{position:fixed;inset:0;display:block;}
  .panel{position:fixed;background:rgba(20,22,30,.82);border:1px solid #333;
    border-radius:8px;padding:10px 12px;font-size:13px;line-height:1.6;z-index:10;}
  #info{top:10px;left:10px;max-width:320px;}
  #info b{color:#7fd1ff;}
  #ctrl{bottom:10px;left:10px;right:10px;display:flex;flex-wrap:wrap;gap:8px;align-items:center;}
  #ctrl .grp{display:flex;gap:6px;align-items:center;background:rgba(20,22,30,.7);
    padding:6px 8px;border-radius:6px;border:1px solid #2a2d3a;}
  button{background:#2a2f42;color:#e8e8ee;border:1px solid #3a4056;border-radius:5px;
    padding:5px 9px;font-size:12px;cursor:pointer;}
  button:hover{background:#3a4160;} button.on{background:#2f7fd1;border-color:#4fa0ff;}
  input[type=range]{vertical-align:middle;}
  #layerlabel{min-width:230px;color:#ffd479;font-size:12px;}
  #tip{position:fixed;pointer-events:none;background:rgba(0,0,0,.85);border:1px solid #4fa0ff;
    border-radius:4px;padding:4px 7px;font-size:12px;z-index:20;display:none;white-space:nowrap;}
  #anim{color:#9effa0;min-width:250px;font-size:12px;}
  .hint{color:#8a90a6;font-size:11px;}
</style>
</head>
<body>
<canvas id="c"></canvas>
<div id="info" class="panel"></div>
<div id="tip"></div>
<div id="ctrl" class="panel">
  <div class="grp">建筑:
    <button id="b_solid" class="on">实体</button>
    <button id="b_trans">半透明</button>
    <button id="b_wire">线框</button>
    <button id="b_hide">隐藏</button>
  </div>
  <div class="grp">
    <button id="b_lmode">层模式: 累积(1~N)</button>
    <input id="slider" type="range" min="0" max="0" value="0" step="1" style="width:180px"/>
    <span id="layerlabel"></span>
  </div>
  <div class="grp">显示:
    <button id="b_lines" class="on">彩色层线</button>
    <button id="b_pilot" class="on">飞手航点</button>
  </div>
  <div class="grp">动画:
    <button id="b_play">▶ 播放</button>
    <button id="b_1x" class="on">1x</button>
    <button id="b_5x">5x</button>
    <button id="b_20x">20x</button>
    <span id="anim"></span>
  </div>
</div>
"""

HTML_TAIL = """
</body>
</html>
"""

# 静态前端逻辑(引用 window.__DATA__)。注意: 不做 python 格式化, 大括号安全。
APP_JS = r"""
(function(){
const D = window.__DATA__;
function b64f32(s){const bin=atob(s);const u=new Uint8Array(bin.length);for(let i=0;i<bin.length;i++)u[i]=bin.charCodeAt(i);return new Float32Array(u.buffer);}
function b64u32(s){const bin=atob(s);const u=new Uint8Array(bin.length);for(let i=0;i<bin.length;i++)u[i]=bin.charCodeAt(i);return new Uint32Array(u.buffer);}

const scene=new THREE.Scene();
scene.background=new THREE.Color(0x0f1117);
const renderer=new THREE.WebGLRenderer({canvas:document.getElementById('c'),antialias:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));
function resize(){renderer.setSize(window.innerWidth,window.innerHeight);camera.aspect=window.innerWidth/window.innerHeight;camera.updateProjectionMatrix();}

const camera=new THREE.PerspectiveCamera(55,1,0.1,5000);
camera.up.set(0,0,1); // z 向上

// ---- 建筑 mesh ----
const mpos=b64f32(D.mesh.pos), mind=b64u32(D.mesh.idx);
const geo=new THREE.BufferGeometry();
geo.setAttribute('position',new THREE.BufferAttribute(mpos,3));
geo.setIndex(new THREE.BufferAttribute(mind,1));
geo.computeVertexNormals();
geo.computeBoundingBox();
const bb=geo.boundingBox;
const center=new THREE.Vector3(); bb.getCenter(center);
const size=new THREE.Vector3(); bb.getSize(size);
const matSolid=new THREE.MeshStandardMaterial({color:0x9aa4b2,metalness:0.05,roughness:0.85,flatShading:false,side:THREE.DoubleSide});
const mesh=new THREE.Mesh(geo,matSolid); scene.add(mesh);

// ---- 光照 ----
scene.add(new THREE.HemisphereLight(0xbfd4ff,0x202028,0.9));
const dir=new THREE.DirectionalLight(0xffffff,0.7); dir.position.set(size.x, -size.y, size.z*1.5); scene.add(dir);
scene.add(new THREE.AmbientLight(0x404050,0.5));

// ---- 地面网格(XY 平面, z=0, 1m) ----
const gridSize=Math.ceil(Math.max(size.x,size.y,size.z)*1.6/2)*2;
const grid=new THREE.GridHelper(gridSize, gridSize, 0x2c3140, 0x20242f);
grid.rotation.x=Math.PI/2; // 放到 XY 平面
grid.position.set(center.x,0,0);
scene.add(grid);

// ---- 原点标记 + 坐标轴 ----
function makeLabel(text,color){
  const cv=document.createElement('canvas');cv.width=256;cv.height=64;const g=cv.getContext('2d');
  g.fillStyle='rgba(0,0,0,0)';g.fillRect(0,0,256,64);
  g.font='bold 34px Microsoft YaHei, sans-serif';g.fillStyle=color;g.textAlign='center';g.textBaseline='middle';
  g.fillText(text,128,34);
  const tex=new THREE.CanvasTexture(cv);tex.needsUpdate=true;
  const sp=new THREE.Sprite(new THREE.SpriteMaterial({map:tex,depthTest:false,transparent:true}));
  sp.scale.set(6,1.5,1);return sp;
}
const originGrp=new THREE.Group();
const disk=new THREE.Mesh(new THREE.CircleGeometry(1.2,32),new THREE.MeshBasicMaterial({color:0xff5555,transparent:true,opacity:0.6}));
originGrp.add(disk); // z=0 平面圆盘
const pole=new THREE.Mesh(new THREE.CylinderGeometry(0.06,0.06,3,12),new THREE.MeshBasicMaterial({color:0xff8888}));
pole.rotation.x=Math.PI/2; pole.position.set(0,0,1.5); originGrp.add(pole);
const AX=Math.max(6, size.x*0.18);
function arrow(dir3,color){return new THREE.ArrowHelper(dir3,new THREE.Vector3(0,0,0),AX,color,AX*0.18,AX*0.09);}
originGrp.add(arrow(new THREE.Vector3(1,0,0),0xff4444)); // x 红 指向建筑
originGrp.add(arrow(new THREE.Vector3(0,1,0),0x44ff44)); // y 绿 左
originGrp.add(arrow(new THREE.Vector3(0,0,1),0x4488ff)); // z 蓝 上
let lb;
lb=makeLabel('起飞点',' #ffaaaa'); lb.position.set(0,0,3.4); originGrp.add(lb);
lb=makeLabel('x→建筑','#ff7777'); lb.position.set(AX+1.5,0,0.3); originGrp.add(lb);
lb=makeLabel('y 左','#77ff77'); lb.position.set(0,AX+1.2,0.3); originGrp.add(lb);
lb=makeLabel('z 上','#7799ff'); lb.position.set(0,0,AX+1.2); originGrp.add(lb);
scene.add(originGrp);

// ---- 路径层 (每层一条 Line, HSV 渐变) ----
const layerLines=[];
const flat=[]; // 动画用: 全路径顺序点
D.layers.forEach(function(L,i){
  const p=b64f32(L.pos);
  const g=new THREE.BufferGeometry();
  g.setAttribute('position',new THREE.BufferAttribute(p,3));
  const col=new THREE.Color(L.c[0],L.c[1],L.c[2]);
  const line=new THREE.Line(g,new THREE.LineBasicMaterial({color:col}));
  line.userData={idx:L.idx,z:L.z,n:p.length/3,color:col};
  scene.add(line); layerLines.push(line);
  for(let k=0;k<p.length;k+=3){flat.push(p[k],p[k+1],p[k+2]);}
});
const flatArr=new Float32Array(flat);

// ---- 飞手降采样航点 (Points + 悬停拾取) ----
let pilotPoints=null, pilotLabels=null;
if(D.pilot){
  const pp=b64f32(D.pilot.pos);
  const g=new THREE.BufferGeometry();
  g.setAttribute('position',new THREE.BufferAttribute(pp,3));
  pilotPoints=new THREE.Points(g,new THREE.PointsMaterial({color:0xffe08a,size:0.5,sizeAttenuation:true}));
  scene.add(pilotPoints);
  pilotLabels=D.pilot.labels;
}

// ---- 无人机动画标记 (锥体) ----
const drone=new THREE.Mesh(new THREE.ConeGeometry(0.6,1.6,16),new THREE.MeshBasicMaterial({color:0x00e5ff}));
drone.visible=false; scene.add(drone);

// ---- 相机初始视角: 斜 45 俯视 ----
const controls=new THREE.OrbitControls(camera,renderer.domElement);
controls.target.copy(center);
const R=Math.max(size.x,size.y,size.z)*1.4;
camera.position.set(center.x - R*0.7, -R*0.9, center.z + R*0.8);
controls.update();
resize();

// ---- 建筑显示模式 ----
function setBuildingMode(m){
  ['b_solid','b_trans','b_wire','b_hide'].forEach(id=>document.getElementById(id).classList.remove('on'));
  document.getElementById('b_'+m).classList.add('on');
  mesh.visible=true; matSolid.wireframe=false; matSolid.transparent=false; matSolid.opacity=1;
  if(m==='solid'){}
  else if(m==='trans'){matSolid.transparent=true;matSolid.opacity=0.42;}
  else if(m==='wire'){matSolid.wireframe=true;}
  else if(m==='hide'){mesh.visible=false;}
  matSolid.needsUpdate=true;
}
document.getElementById('b_solid').onclick=()=>setBuildingMode('solid');
document.getElementById('b_trans').onclick=()=>setBuildingMode('trans');
document.getElementById('b_wire').onclick=()=>setBuildingMode('wire');
document.getElementById('b_hide').onclick=()=>setBuildingMode('hide');

// ---- 层滑块 ----
const nLayers=layerLines.length;
const slider=document.getElementById('slider');
slider.max=nLayers-1; slider.value=nLayers-1; // 默认显示全部层(累积到顶)
let cumulative=true; // 累积 vs 单层
let showLines=true;  // 彩色层线开关
function updateLayers(){
  const N=parseInt(slider.value);
  for(let i=0;i<nLayers;i++){
    const ln=layerLines[i];
    const vis = cumulative ? (i<=N) : (i===N);
    ln.visible = showLines && vis;
    ln.material.linewidth=1;
  }
  // 高亮当前层
  const cur=layerLines[N];
  if(showLines) cur.visible=true;
  const c=cur.userData;
  document.getElementById('layerlabel').innerHTML=
    '第 '+(N+1)+'/'+nLayers+' 层 &nbsp; z='+c.z.toFixed(2)+' m &nbsp; 航点 '+c.n;
}
slider.oninput=updateLayers;
document.getElementById('b_lmode').onclick=function(){
  cumulative=!cumulative;
  this.textContent = cumulative ? '层模式: 累积(1~N)' : '层模式: 仅第N层';
  updateLayers();
};
document.getElementById('b_lines').onclick=function(){
  showLines=!showLines; this.classList.toggle('on',showLines); updateLayers();
};
document.getElementById('b_pilot').onclick=function(){
  if(!pilotPoints){return;}
  pilotPoints.visible=!pilotPoints.visible; this.classList.toggle('on',pilotPoints.visible);
};
updateLayers();

// ---- 动画 ----
let playing=false, animIdx=0, speed=1;
const bplay=document.getElementById('b_play');
bplay.onclick=function(){playing=!playing;drone.visible=true;this.textContent=playing?'⏸ 暂停':'▶ 播放';};
function setSpeed(s,id){speed=s;['b_1x','b_5x','b_20x'].forEach(x=>document.getElementById(x).classList.remove('on'));document.getElementById(id).classList.add('on');}
document.getElementById('b_1x').onclick=()=>setSpeed(1,'b_1x');
document.getElementById('b_5x').onclick=()=>setSpeed(5,'b_5x');
document.getElementById('b_20x').onclick=()=>setSpeed(20,'b_20x');
const nPts=flatArr.length/3;
// 预算每层的累计点数, 用于动画时显示层号
const layerCum=[]; let acc=0;
D.layers.forEach(L=>{acc+=b64f32(L.pos).length/3;layerCum.push(acc);});
function layerOfIdx(i){for(let k=0;k<layerCum.length;k++){if(i<layerCum[k])return k;}return layerCum.length-1;}

// ---- 悬停拾取 (飞手航点) ----
const ray=new THREE.Raycaster(); ray.params.Points.threshold=0.6;
const mouse=new THREE.Vector2(); const tip=document.getElementById('tip');
renderer.domElement.addEventListener('mousemove',function(e){
  if(!pilotPoints){return;}
  mouse.x=(e.clientX/window.innerWidth)*2-1; mouse.y=-(e.clientY/window.innerHeight)*2+1;
  ray.setFromCamera(mouse,camera);
  const hit=ray.intersectObject(pilotPoints);
  if(hit.length){
    const idx=hit[0].index; const L=pilotLabels[idx];
    tip.style.display='block'; tip.style.left=(e.clientX+12)+'px'; tip.style.top=(e.clientY+8)+'px';
    tip.innerHTML='航点 层'+L[0]+' #'+L[1]+'<br>x='+L[2]+' y='+L[3]+' z='+L[4];
  } else { tip.style.display='none'; }
});

// ---- 信息面板 ----
document.getElementById('info').innerHTML=D.infoHtml;

// ---- 渲染循环 ----
let last=performance.now();
function tick(now){
  const dt=(now-last)/1000; last=now;
  if(playing && nPts>1){
    animIdx += speed*dt* (nPts/60.0); // 基准 ~60 点/秒 @1x
    if(animIdx>=nPts){animIdx=0;}
    const i=Math.floor(animIdx)*3;
    drone.position.set(flatArr[i],flatArr[i+1],flatArr[i+2]);
    const li=layerOfIdx(Math.floor(animIdx));
    document.getElementById('anim').innerHTML=
      '第'+(li+1)+'层 &nbsp; ('+flatArr[i].toFixed(1)+', '+flatArr[i+1].toFixed(1)+', '+flatArr[i+2].toFixed(1)+')';
  }
  controls.update();
  renderer.render(scene,camera);
  requestAnimationFrame(tick);
}
window.addEventListener('resize',resize);
requestAnimationFrame(tick);
})();
"""


# ---------------------------------------------------------------------------
def build_info_html(meta, model_name, n_layers, total_pts, total_dist, height, n_pilot):
    note = meta.get("coordinate_system_note", "")
    return (
        "<b>喷涂路径查看器</b><br>"
        "模型: " + model_name + "<br>"
        "层数: <b>" + str(n_layers) + "</b> &nbsp; 全量航点: <b>" + str(total_pts) + "</b><br>"
        + ("飞手航点: <b>" + str(n_pilot) + "</b><br>" if n_pilot else "")
        + "总里程: <b>" + ("%.0f" % total_dist) + " m</b> &nbsp; 建筑高: <b>" + ("%.1f" % height) + " m</b><br>"
        "<span class='hint'>坐标系: " + note + "</span><br>"
        "<span class='hint'>路径应贴在建筑外侧约 spray_distance 米处; 拖动下方滑块逐层查看, ▶ 播放沿路径飞行。</span>"
    )


def main():
    ap = argparse.ArgumentParser(
        description="STL抽稀 + 航点 -> 单文件离线HTML 3D查看器",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--stl", required=True, help="展示用 STL(应与规划模型一致)")
    ap.add_argument("--csv", required=True, help="waypoints_full.csv (全量航点, 画路径)")
    ap.add_argument("--pilot-csv", default=None, help="waypoints_pilot.csv (可选, 标注编号航点)")
    ap.add_argument("--meta", required=True, help="export_meta.json (读部署参数用于摆放STL)")
    ap.add_argument("--target-faces", type=int, default=150000, help="抽稀目标面数")
    ap.add_argument("--out", default="viewer.html", help="输出 HTML")
    ap.add_argument("--max-mb", type=float, default=40.0, help="HTML 上限(MB), 超出自动降面数重试")
    args = ap.parse_args()

    with open(args.meta) as f:
        meta = json.load(f)

    three_js = load_vendor_js("three.min.js", THREE_URL)
    orbit_js = load_vendor_js("OrbitControls.js", ORBIT_URL)

    layers, total_pts, total_dist = read_layers(args.csv)
    pilot = read_pilot(args.pilot_csv) if args.pilot_csv else None
    n_layers = len(layers)
    height = max((L["z"] for L in layers), default=0.0)
    model_name = os.path.basename(args.stl)

    target = args.target_faces
    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)

    for attempt in range(4):
        verts, tris, bbox_min, bbox_max, orig_faces, mq = decimate_mesh(args.stl, target)
        placed, tm = place_vertices(verts, bbox_min, bbox_max, meta)

        # 保存 decimated_preview.stl (局部系, 供肉眼检查)
        try:
            import open3d as o3d
            preview_path = os.path.join(out_dir, "decimated_preview.stl")
            mq.compute_vertex_normals()
            o3d.io.write_triangle_mesh(preview_path, mq)
            print("[viewer] saved preview -> %s (faces=%d)" % (preview_path, len(tris)))
        except Exception as e:
            print("[viewer] warn: 保存 decimated_preview.stl 失败: %s" % e)

        # HSV 层色: 第1层红(0) -> 末层紫(0.83)
        layer_data = []
        for i, L in enumerate(layers):
            h = (i / max(1, n_layers - 1)) * 0.83
            r, g, b = colorsys.hsv_to_rgb(h, 1.0, 1.0)
            layer_data.append({"idx": L["idx"], "z": L["z"],
                               "c": [round(r, 4), round(g, 4), round(b, 4)],
                               "pos": b64_f32(L["pos"].reshape(-1))})

        data = {
            "mesh": {"pos": b64_f32(placed.astype(np.float32).reshape(-1)),
                     "idx": b64_u32(tris.reshape(-1))},
            "layers": layer_data,
            "pilot": ({"pos": b64_f32(pilot["pos"].reshape(-1)), "labels": pilot["labels"]}
                      if pilot else None),
            "infoHtml": build_info_html(meta, model_name, n_layers, total_pts,
                                        total_dist, height, len(pilot["labels"]) if pilot else 0),
        }
        html = build_html(three_js, orbit_js, data)
        mb = len(html.encode("utf-8")) / 1e6
        print("[viewer] HTML size = %.1f MB (target_faces=%d)" % (mb, target))
        if mb <= args.max_mb or target <= 20000:
            break
        target = int(target * 0.6)
        print("[viewer] 超过 %.0fMB, 降 target-faces 到 %d 重试" % (args.max_mb, target))

    with open(args.out, "w", encoding="utf-8") as f:
        f.write(html)
    print("[viewer] DONE -> %s (%.1f MB)  层数=%d 全量航点=%d%s"
          % (args.out, mb, n_layers, total_pts,
             (" 飞手航点=%d" % len(pilot["labels"])) if pilot else ""))
    print("[viewer] 断网状态下用 Chrome/Edge 双击打开即可。")


if __name__ == "__main__":
    main()
