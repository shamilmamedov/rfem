from panda3d_viewer import Viewer, ViewerConfig
from pinocchio.visualize import Panda3dVisualizer, MeshcatVisualizer
import time
import pinocchio as pin

PANDA3D_CONFIG = ViewerConfig()
PANDA3D_CONFIG.enable_antialiasing(True, multisamples=4)
PANDA3D_CONFIG.enable_shadow(True)
PANDA3D_CONFIG.show_axes(True)
PANDA3D_CONFIG.show_grid(False)
PANDA3D_CONFIG.show_floor(False)
PANDA3D_CONFIG.enable_spotlight(False)
PANDA3D_CONFIG.enable_hdr(False)


def visualize_elastic_pendulum(q, dt, n_replays, urdf_path, visualizer='panda3d'):
    # Let pinocchio to build model, collision and visual models from URDF
    m, cm, vm = pin.buildModelsFromUrdf(urdf_path)

    assert q.shape[1] == m.nq

    if visualizer == 'panda3d':
        viz = Panda3dVisualizer(m, cm, vm)
        viewer = Viewer(config=PANDA3D_CONFIG)
        viewer.set_background_color(((173, 216, 230)))
        viz.initViewer(viewer=viewer)
        viz.loadViewerModel(group_name='elastic-pendulum')
    elif visualizer == 'meshcat':
        viz = MeshcatVisualizer(m, cm, vm)
        viz.initViewer()
        viz.loadViewerModel()
    else: 
        raise ValueError

    for _ in range(n_replays):
        viz.display(q[0, :])
        time.sleep(2)
        viz.play(q[1:, :].T, dt)
        time.sleep(1)