from pxr import Usd, PhysxSchema

usd_path = "assets/roarm_m3/usd/roarm_m3.usd"
stage = Usd.Stage.Open(usd_path)

prim = stage.GetPrimAtPath("/World/roarm_m3")
if prim:
    api = PhysxSchema.PhysxArticulationRootAPI.Apply(prim)
    print("PhysxSchema.ArticulationRootAPI 추가 완료")
    stage.GetRootLayer().Save()
else:
    print("Prim 경로가 올바르지 않습니다.")
