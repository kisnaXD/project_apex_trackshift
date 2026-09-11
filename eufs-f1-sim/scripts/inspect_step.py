#!/usr/bin/env python3
"""Inspect STEP assembly tree via FreeCAD headless (run with freecadcmd)."""
import sys
from pathlib import Path

STEP = Path(sys.argv[1] if len(sys.argv) > 1 else '/tmp/mercedes/Assem step.STEP')

import FreeCAD as App
import Import

doc = App.newDocument('inspect')
Import.insert(str(STEP), doc.Name)
doc.recompute()

print('FreeCAD', App.Version())
print('objects', len(doc.Objects))
for obj in doc.Objects:
    bb = obj.Shape.BoundBox if hasattr(obj, 'Shape') and not obj.Shape.isNull() else None
    if bb:
        ext = (bb.XLength, bb.YLength, bb.ZLength)
        cen = (bb.Center.x, bb.Center.y, bb.Center.z)
    else:
        ext = cen = None
    print(f'{obj.Label!r} type={obj.TypeId} ext={ext} center={cen}')

if doc.Objects:
    all_bb = doc.Objects[0].Shape.BoundBox
    for obj in doc.Objects[1:]:
        if hasattr(obj, 'Shape') and not obj.Shape.isNull():
            all_bb.add(obj.Shape.BoundBox)
    print('ASSEMBLY_EXT', all_bb.XLength, all_bb.YLength, all_bb.ZLength)
