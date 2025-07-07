from dm_control import mjcf
from mjcf import from_urdf
model = from_urdf.load_urdf('/data1/linmin/RoboticDogPatrol/rm_65_b_description/urdf/rm_65_b_description.urdf')

# 保存为 MJCF XML 文件
with open("rm65.xml", "w") as f:
    f.write(model.to_xml_string(pretty=True))