from isaacsim import SimulationApp

# pxr / omni 모듈을 import하기 전에 Isaac Sim Kit을 먼저 시작해야 한다.
simulation_app = SimulationApp({"headless": True})

from pathlib import Path

from pxr import Usd, UsdGeom, UsdShade


# ============================================================
# 경로
# ============================================================
THIS_DIR = Path(__file__).resolve().parent
MISSION_DIR = THIS_DIR.parent

USD_PATH = (
    MISSION_DIR
    / "usd"
    / "Collected_m0609_camera_cube"
    / "m0609_camera_cube.usd"
)

CUBE_PRIM_PATH = "/World/red_block/Cube"


# ============================================================
# USD 열기
# ============================================================
stage = Usd.Stage.Open(str(USD_PATH))

if stage is None:
    raise RuntimeError(f"USD를 열 수 없습니다: {USD_PATH}")

cube = stage.GetPrimAtPath(CUBE_PRIM_PATH)

if not cube.IsValid():
    raise RuntimeError(f"Cube Prim을 찾을 수 없습니다: {CUBE_PRIM_PATH}")


# ============================================================
# 기본 정보
# ============================================================
print("=" * 70)
print("CUBE APPEARANCE INSPECTION")
print("=" * 70)

print(f"USD       : {USD_PATH}")
print(f"Prim Path : {cube.GetPath()}")
print(f"Type      : {cube.GetTypeName()}")
print()


# ============================================================
# Display Color
# ============================================================
print("[1] DISPLAY COLOR")

gprim = UsdGeom.Gprim(cube)

if gprim:
    display_color_attr = gprim.GetDisplayColorAttr()
    display_color = display_color_attr.Get()

    print(f"displayColor = {display_color}")
else:
    print("이 Prim 자체는 UsdGeom.Gprim이 아닙니다.")

print()


# ============================================================
# Material Binding
# ============================================================
print("[2] MATERIAL BINDING")

binding_api = UsdShade.MaterialBindingAPI(cube)
material, relationship = binding_api.ComputeBoundMaterial()

if material:
    print(f"Material = {material.GetPath()}")

    material_prim = material.GetPrim()

    print()
    print("  Material outputs:")

    outputs = material.GetOutputs()

    if not outputs:
        print("    없음")
    else:
        for output in outputs:
            print(
                f"    {output.GetBaseName():20s}"
                f" type={output.GetTypeName()}"
            )

    print()
    print("  Material descendants:")

    found_shader = False

    for prim in Usd.PrimRange(material_prim):
        if prim == material_prim:
            continue

        print(
            f"    {prim.GetPath()}"
            f"  type={prim.GetTypeName()}"
        )

        if prim.IsA(UsdShade.Shader):
            found_shader = True

            shader = UsdShade.Shader(prim)

            shader_id = shader.GetIdAttr().Get()

            print(f"      shader id = {shader_id}")

            for input_attr in shader.GetInputs():
                value = input_attr.Get()

                print(
                    f"      input "
                    f"{input_attr.GetBaseName():24s}"
                    f" = {value}"
                )

    if not found_shader:
        print("    Shader Prim 없음")

else:
    print("직접 또는 상속된 Material Binding 없음")

print()

print()


# ============================================================
# 하위 Prim
# ============================================================
print("[3] CHILD PRIMS")

children = list(cube.GetChildren())

if not children:
    print("하위 Prim 없음")
else:
    for child in children:
        print(
            f"{child.GetPath()} "
            f"(type={child.GetTypeName()})"
        )

print()


# ============================================================
# 전체 하위 Material 조사
# ============================================================
print("[4] DESCENDANT MATERIALS")

found = False

for prim in Usd.PrimRange(cube):
    if prim == cube:
        continue

    binding_api = UsdShade.MaterialBindingAPI(prim)
    material, _ = binding_api.ComputeBoundMaterial()

    if material:
        found = True
        print(
            f"{prim.GetPath()} "
            f"→ {material.GetPath()}"
        )

if not found:
    print("하위 Prim에서도 Material Binding 없음")

print()
print("=" * 70)
print("INSPECTION COMPLETE")
print("=" * 70)

simulation_app.close()