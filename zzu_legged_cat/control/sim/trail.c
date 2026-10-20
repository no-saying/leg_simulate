#include "trail.h"

void trail_reset(OdomTrail *trail)
{
    trail->count = 0;
    trail->next = 0;
    trail->last_t = -1.0;
}

void trail_push(OdomTrail *trail, double t, float ox, float oy, float bx, float by)
{
    /* 约 20 Hz 的轨迹点足够看回原点表现，也不会挤爆 mjvScene。 */
    if (trail->last_t >= 0.0 && (t - trail->last_t) < 0.05)
        return;

    trail->last_t = t;
    trail->odom_x[trail->next] = ox;
    trail->odom_y[trail->next] = oy;
    trail->body_x[trail->next] = bx;
    trail->body_y[trail->next] = by;
    trail->next = (trail->next + 1) % ODOM_TRAIL_MAX;
    if (trail->count < ODOM_TRAIL_MAX) trail->count++;
}

static void add_sphere(mjvScene *scn, const mjtNum pos[3], mjtNum radius, const float rgba[4])
{
    if (scn->ngeom >= scn->maxgeom) return;

    mjvGeom *g = scn->geoms + scn->ngeom;
    mjtNum size[3] = {radius, 0.0, 0.0};
    mjv_initGeom(g, mjGEOM_SPHERE, size, pos, NULL, rgba);
    scn->ngeom++;
}

void trail_add_scene(const OdomTrail *trail, mjvScene *scn)
{
    if (!scn || trail->count <= 0) return;

    {
        const mjtNum pos[3] = {0.0, 0.0, 0.02};
        const float rgba[4] = {1.0f, 1.0f, 0.2f, 0.95f};
        add_sphere(scn, pos, 0.035, rgba);
    }

    {
        const mjtNum corners[4][2] = {
            { 0.10,  0.10}, { 0.10, -0.10},
            {-0.10,  0.10}, {-0.10, -0.10}
        };
        const float rgba[4] = {0.2f, 1.0f, 0.4f, 0.80f};
        for (int k = 0; k < 4; k++) {
            const mjtNum pos[3] = {corners[k][0], corners[k][1], 0.015};
            add_sphere(scn, pos, 0.018, rgba);
        }
    }

    int start = (trail->count < ODOM_TRAIL_MAX) ? 0 : trail->next;
    int step = (trail->count > 400) ? (trail->count + 399) / 400 : 1;

    for (int k = 0; k < trail->count; k += step) {
        int idx = (start + k) % ODOM_TRAIL_MAX;
        const mjtNum pos[3] = {trail->odom_x[idx], trail->odom_y[idx], 0.018};
        const float rgba[4] = {0.15f, 0.85f, 1.0f, 0.85f};
        add_sphere(scn, pos, 0.012, rgba);
    }

    if (trail->count > 0) {
        int last = (trail->next - 1 + ODOM_TRAIL_MAX) % ODOM_TRAIL_MAX;
        const mjtNum pos[3] = {trail->odom_x[last], trail->odom_y[last], 0.04};
        const float rgba[4] = {0.1f, 0.35f, 1.0f, 1.0f};
        add_sphere(scn, pos, 0.04, rgba);
    }

    for (int k = 0; k < trail->count; k += step * 2) {
        int idx = (start + k) % ODOM_TRAIL_MAX;
        const mjtNum pos[3] = {trail->body_x[idx], trail->body_y[idx], 0.012};
        const float rgba[4] = {1.0f, 0.25f, 0.75f, 0.55f};
        add_sphere(scn, pos, 0.010, rgba);
    }

    if (trail->count > 0) {
        int last = (trail->next - 1 + ODOM_TRAIL_MAX) % ODOM_TRAIL_MAX;
        const mjtNum pos[3] = {trail->body_x[last], trail->body_y[last], 0.035};
        const float rgba[4] = {1.0f, 0.15f, 0.15f, 0.95f};
        add_sphere(scn, pos, 0.035, rgba);
    }
}

int trail_count(const OdomTrail *trail)
{
    return trail->count;
}
