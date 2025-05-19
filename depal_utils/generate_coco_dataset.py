#!/usr/bin/env python3
from __future__ import annotations
import json, re, shutil
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple, Union
import numpy as np, cv2
from PIL import Image

INCLUDE_CLASSES = {"box"}

class COCODict:
    def __init__(self):
        self.images, self.annotations, self.categories = [], [], []
    def to_json(self):
        return json.dumps({"images": self.images, "annotations": self.annotations, "categories": self.categories}, indent=2)

class CategoryRegister:
    def __init__(self):
        self._map: Dict[str, int] = {}
    def get(self, name: str) -> int:
        k = name.lower()
        if k not in self._map:
            self._map[k] = len(self._map) + 1
        return self._map[k]
    def build(self):
        return [{"id": i, "name": n, "supercategory": "none"} for n, i in self._map.items()]

IDX_RE = re.compile(r"_(\d{4})\.")

def key_for(p: Path) -> str:
    m = IDX_RE.search(p.name)
    if not m:
        raise ValueError(p.name)
    return f"{p.parent.name}_{m.group(1)}"  # e.g. img5_0000

def poly_from_mask(m: np.ndarray):
    cnts, _ = cv2.findContours((m * 255).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return [c.squeeze(1).astype(float).flatten().tolist() for c in cnts if c.shape[0] >= 3]

def bbox_from_mask(m: np.ndarray):
    ys, xs = np.where(m)
    x0, y0, x1, y1 = xs.min(), ys.min(), xs.max(), ys.max()
    return int(x0), int(y0), int(x1 - x0 + 1), int(y1 - y0 + 1)

def parse_color(k: str):
    return tuple(int(x) for x in k.strip("() ").split(","))

def load_mapping(path: Path):
    js = json.loads(path.read_text())
    if isinstance(js, dict) and "mapping" in js:
        js = js["mapping"]
    out: Dict[Union[int, Tuple[int, int, int, int]], str] = {}
    if isinstance(js, list):
        if all(isinstance(x, dict) for x in js):
            for e in js:
                iid = e.get("instanceId") or e.get("instance_id") or e.get("id")
                if iid is None:
                    continue
                out[int(iid)] = e.get("class") or e.get("name") or e.get("category") or str(iid)
        else:
            for iid, name in enumerate(js):
                out[iid] = str(name)
    elif isinstance(js, dict):
        val = next(iter(js.values()))
        if isinstance(val, str):
            out = {int(k): v for k, v in js.items() if k.isdigit()}
        else:
            for k, v in js.items():
                out[parse_color(k)] = v.get("class") or v.get("name") or v.get("category")
    if not out:
        raise ValueError(path)
    return out

def add_annos(mask_np, mapping, image_id, anno_id, catreg, annos):
    inc = {c.lower() for c in INCLUDE_CLASSES}
    if mask_np.ndim == 2:
        for iid in np.unique(mask_np):
            if iid == 0:
                continue
            cname = mapping.get(int(iid))
            if not cname or cname.lower() not in inc:
                continue
            cat_id = catreg.get(cname)
            bin_mask = (mask_np == iid).astype(np.uint8)
            segm = poly_from_mask(bin_mask)
            if not segm:
                continue
            annos.append({"id": anno_id, "image_id": image_id, "category_id": cat_id,
                          "bbox": list(bbox_from_mask(bin_mask)),
                          "area": int(bin_mask.sum()), "segmentation": segm, "iscrowd": 0})
            anno_id += 1
    else:
        if mask_np.shape[2] == 3:
            mask_np = np.dstack((mask_np, 255 * np.ones(mask_np.shape[:2], dtype=mask_np.dtype)))
        colors = np.unique(mask_np.reshape(-1, 4), axis=0)
        for col in colors:
            col_t = tuple(int(x) for x in col)
            cname = mapping.get(col_t)
            if not cname or cname.lower() not in inc:
                continue
            cat_id = catreg.get(cname)
            bin_mask = np.all(mask_np == col, axis=-1).astype(np.uint8)
            if bin_mask.sum() == 0:
                continue
            segm = poly_from_mask(bin_mask)
            if not segm:
                continue
            annos.append({"id": anno_id, "image_id": image_id, "category_id": cat_id,
                          "bbox": list(bbox_from_mask(bin_mask)),
                          "area": int(bin_mask.sum()), "segmentation": segm, "iscrowd": 0})
            anno_id += 1
    return anno_id

def main(src: Path, dst: Path):
    imgs_out = dst / "images"
    imgs_out.mkdir(parents=True, exist_ok=True)
    coco, catreg = COCODict(), CategoryRegister()
    bundles = defaultdict(dict)
    for p in src.rglob("instance_segmentation_*[0-9].png"):
        bundles[key_for(p)]["mask"] = p
    for p in src.rglob("rgb_*[0-9].png"):
        bundles[key_for(p)]["rgb"] = p
    for p in src.rglob("instance_segmentation_semantics_mapping_*[0-9].json"):
        bundles[key_for(p)]["map"] = p
    anno_id = image_id = 1
    for k, b in sorted(bundles.items()):
        if not {"rgb", "mask", "map"}.issubset(b):
            continue
        rgb, mask_p, map_p = b["rgb"], b["mask"], b["map"]
        with Image.open(rgb) as im:
            w, h = im.size
        new_name = f"{rgb.parent.name}_{rgb.name}"
        coco.images.append({"id": image_id, "file_name": new_name, "width": w, "height": h})
        anno_id = add_annos(np.array(Image.open(mask_p)), load_mapping(map_p), image_id, anno_id, catreg, coco.annotations)
        shutil.copy(rgb, imgs_out / new_name)
        image_id += 1
    coco.categories = catreg.build()
    (dst / "annotations.json").write_text(coco.to_json(), encoding="utf-8")
    print(f"images {len(coco.images)} ann {len(coco.annotations)} cat {len(coco.categories)}")

if __name__ == "__main__":
    root = Path(__file__).resolve().parent.parent
    main(root / "output", root / "dataset")
