#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Exercise the actual GUI camera updates without creating a desktop window. */
#include "../../control/app/app_gui.c"

int main(void)
{
    char error[1024];
    Simulation sim = {0};
    sim.model = mj_loadXML("model/v3/COD-2026RoboMaster-Balance-v3.xml",
                           NULL, error, sizeof(error));
    if (!sim.model) {
        fprintf(stderr, "%s\n", error);
        return 1;
    }
    sim.data = mj_makeData(sim.model);
    assert(sim.data);
    sim.state.quat[0] = 1.0;
    GuiRuntime gui = {0};
    gui.track_body_id = mj_name2id(sim.model, mjOBJ_BODY, "base_link");
    assert(gui.track_body_id > 0);
    mjv_defaultCamera(&gui.camera);
    mjvScene scene;
    mjv_defaultScene(&scene);

    for (int preset = 0; preset < 6; ++preset) {
        gui.camera_preset = preset;
        apply_camera_preset(&gui);
        for (int step = 0; step < 3; ++step) {
            sim.data->qpos[0] = 3.0*step;
            sim.data->qpos[1] = -2.0*step;
            sim.data->qpos[2] = 0.7*step;
            /* Scene props can dominate the world COM while the robot moves. */
            for (int j = 1; j < sim.model->njnt; ++j) {
                if (sim.model->jnt_type[j] != mjJNT_FREE) continue;
                int address = sim.model->jnt_qposadr[j];
                sim.data->qpos[address] = 1000.0*(step+1);
            }
            mj_forward(sim.model, sim.data);
            update_camera_follow(&gui, &sim);
            mjtNum expected[3];
            mju_copy3(expected, sim.data->xpos + 3*gui.track_body_id);
            expected[2] += preset == 5 ? 0.15 : 0.10;
            if (preset == 5) expected[0] += 0.5;
            assert(gui.camera.type == mjCAMERA_FREE);
            assert(mju_dist3(gui.camera.lookat, expected) < 1e-10);
            mjv_updateCamera(sim.model, sim.data, &gui.camera, &scene);
            assert(mju_dist3(gui.camera.lookat, expected) < 1e-10);
            mjtNum eye[3];
            for (int k = 0; k < 3; ++k)
                eye[k] = 0.5*(scene.camera[0].pos[k] + scene.camera[1].pos[k]);
            assert(fabs(mju_dist3(eye, expected)-gui.camera.distance) < 1e-5);
        }
    }
    mj_deleteData(sim.data);
    mj_deleteModel(sim.model);
    puts("PASS: 6 camera presets follow XYZ and retain distance as scene props move");
    return 0;
}
