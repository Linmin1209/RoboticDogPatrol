import mujoco
import mujoco.viewer
import time

# 使用原始字符串处理路径
model = mujoco.MjModel.from_xml_path("/data1/linmin/RoboticDogPatrol/patrol_dog/b2_description_mujoco/xml/scene.xml")
data = mujoco.MjData(model)

# 使用交互式查看器
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 设置相机参数（可选）
    viewer.cam.distance = 5  # 相机距离
    viewer.cam.azimuth = 90  # 方位角
    viewer.cam.elevation = -20  # 仰角
    
    # 模拟循环
    while viewer.is_running():
        step_start = time.time()
        
        # 物理步进
        mujoco.mj_step(model, data)
        
        # 同步查看器
        viewer.sync()
        
        # 控制步长（可选）
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)