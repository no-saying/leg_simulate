/* Training-only contact measurements. No controller or actor observes these. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <mujoco/mujoco.h>
#include <omp.h>

typedef struct {
    const mjModel *model;
    int count, threads, wheels[2], surfaces[3];
    mjData **data;
    mjtNum **jac;
} SpinContacts;

void spin_contacts_destroy(SpinContacts *c)
{
    if (!c) return;
    for (int t = 0; t < c->threads; ++t) {
        if (c->data) mj_deleteData(c->data[t]);
        if (c->jac) free(c->jac[t]);
    }
    free(c->data);
    free(c->jac);
    free(c);
}

SpinContacts *spin_contacts_create(const mjModel *m, int count, int threads)
{
    if (!m || count < 1 || threads < 1) return NULL;
    SpinContacts *c = calloc(1, sizeof(*c));
    if (!c) return NULL;
    c->model = m; c->count = count; c->threads = threads < count ? threads : count;
    c->wheels[0] = mj_name2id(m, mjOBJ_BODY, "Left_Wheel_link");
    c->wheels[1] = mj_name2id(m, mjOBJ_BODY, "Right_Wheel_link");
    const char *names[3] = {"floor", "spin_ramp", "spin_upper_flat"};
    for (int k = 0; k < 3; ++k) c->surfaces[k] = mj_name2id(m, mjOBJ_GEOM, names[k]);
    if (c->wheels[0] < 0 || c->wheels[1] < 0 || c->surfaces[0] < 0) {
        spin_contacts_destroy(c); return NULL;
    }
    c->data = calloc(c->threads, sizeof(mjData *));
    c->jac = calloc(c->threads, sizeof(mjtNum *));
    if (!c->data || !c->jac) { spin_contacts_destroy(c); return NULL; }
    for (int t = 0; t < c->threads; ++t) {
        c->data[t] = mj_makeData(m);
        c->jac[t] = calloc(3*m->nv, sizeof(mjtNum));
        if (!c->data[t] || !c->jac[t]) { spin_contacts_destroy(c); return NULL; }
    }
    return c;
}

/* Per side: total normal load, load-weighted tangential slip speed,
 * load-weighted contact height, number of loaded terrain contacts. */
void spin_contacts_query(SpinContacts *c, const mjtNum *states, mjtNum *out)
{
    const mjModel *m = c->model;
    int ns = mj_stateSize(m, mjSTATE_INTEGRATION);
    #pragma omp parallel for num_threads(c->threads) schedule(static)
    for (int i = 0; i < c->count; ++i) {
        int t = omp_get_thread_num();
        mjData *d = c->data[t];
        mjtNum *jac = c->jac[t], *row = out+8*i;
        memset(row, 0, 8*sizeof(mjtNum));
        mj_resetData(m, d);
        mj_setState(m, d, states+(size_t)i*ns, mjSTATE_INTEGRATION);
        mj_forward(m, d);
        for (int k = 0; k < d->ncon; ++k) {
            const mjContact *contact = d->contact+k;
            for (int side = 0; side < 2; ++side) {
                int slot = m->geom_bodyid[contact->geom[0]] == c->wheels[side] ? 0 :
                           m->geom_bodyid[contact->geom[1]] == c->wheels[side] ? 1 : -1;
                if (slot < 0) continue;
                int other = contact->geom[1-slot], terrain = 0;
                for (int j = 0; j < 3; ++j) if (other == c->surfaces[j]) terrain = 1;
                if (!terrain) continue;
                mjtNum force[6], velocity[3], support[3];
                mj_contactForce(m, d, k, force);
                if (force[0] <= 1e-6) continue;
                mj_jac(m, d, jac, NULL, contact->pos, c->wheels[side]);
                mju_mulMatVec(velocity, jac, d->qvel, 3, m->nv);
                mj_jac(m, d, jac, NULL, contact->pos, m->geom_bodyid[other]);
                mju_mulMatVec(support, jac, d->qvel, 3, m->nv);
                mju_subFrom3(velocity, support);
                mjtNum normal = mju_dot3(velocity, contact->frame);
                for (int j = 0; j < 3; ++j) velocity[j] -= normal*contact->frame[j];
                mjtNum *wheel = row+4*side;
                wheel[0] += force[0];
                wheel[1] += force[0]*mju_norm3(velocity);
                wheel[2] += force[0]*contact->pos[2];
                wheel[3] += 1;
            }
        }
        for (int side = 0; side < 2; ++side) if (row[4*side] > 0) {
            row[4*side+1] /= row[4*side];
            row[4*side+2] /= row[4*side];
        }
    }
}
