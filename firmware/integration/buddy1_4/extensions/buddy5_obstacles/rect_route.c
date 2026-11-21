#include "extensions/buddy5_obstacles/rect_route.h"
#include <math.h>
#include <string.h>

#define DEG (3.14159265358979323846f / 180.0f)

/* Reverse needed so a full in-place rotation (conservative: whole circle of
 * the farthest corner) keeps MARGIN from one sonar point. Range is shortened
 * by its allowance and the bearing moved 4 deg towards the centreline. */
static float point_need(unsigned range_mm, int angle_dd, float reach) {
    float rho = (float)range_mm - B5R_RANGE_ALLOW_MM;
    if (rho < 0) rho = 0;
    float a = fabsf((float)angle_dd) / 10.0f, lo = a - B5R_ANGLE_ALLOW_DD / 10.0f;
    float hi = a + B5R_ANGLE_ALLOW_DD / 10.0f;
    if (lo < 0) lo = 0;
    if (hi > 90) hi = 90;
    float x = rho * sinf(lo * DEG), y = B5R_SONAR_AHEAD_MM + rho * cosf(hi * DEG);
    if (x >= reach) return -1000.0f;
    return sqrtf(reach * reach - x * x) - y;
}
static float point_clear(unsigned range_mm, int angle_dd, float backoff, float radius) {
    float rho = (float)range_mm - B5R_RANGE_ALLOW_MM;
    if (rho < 0) rho = 0;
    float a = fabsf((float)angle_dd) / 10.0f, lo = a - B5R_ANGLE_ALLOW_DD / 10.0f;
    float hi = a + B5R_ANGLE_ALLOW_DD / 10.0f;
    if (lo < 0) lo = 0;
    if (hi > 90) hi = 90;
    float x = rho * sinf(lo * DEG), y = B5R_SONAR_AHEAD_MM + rho * cosf(hi * DEG) + backoff;
    return sqrtf(x * x + y * y) - radius;
}
bool b5_rect_apply_range_bounds (B5RectPlan * p_plan, unsigned nearest_mm,
                                 unsigned farthest_mm)
{
    float clearance_mm = point_clear(nearest_mm, 0, p_plan->backoff_mm,
                                     p_plan->sweep_radius_mm);
    float reach_mm = p_plan->sweep_radius_mm + B5R_MARGIN_MM;
    float backoff_mm = ceilf(point_need(nearest_mm, 0, reach_mm));
    p_plan->first_turn_clear_mm =
        fminf(p_plan->first_turn_clear_mm, clearance_mm);
    p_plan->reverse_needed_mm = fmaxf(p_plan->reverse_needed_mm, backoff_mm);
    p_plan->turn_clearance_ok = p_plan->first_turn_clear_mm >= B5R_MARGIN_MM;
    float extra_pass_mm =
        fmaxf(0.0f, (float)farthest_mm - (float)p_plan->forward_range_mm);
    p_plan->far_y_mm += extra_pass_mm;
    p_plan->pass_y_mm += extra_pass_mm;
    p_plan->forward_mm = ceilf(p_plan->pass_y_mm + p_plan->backoff_mm);
    return p_plan->forward_mm <= 1000.0f;
}

bool b5_rect_plan(const b5_scan_t *scan, unsigned range, B5RectPlan *p) {
    return b5_rect_plan_ex(scan, range, B5R_OBST_DEPTH_MM, true, p);
}
bool b5_rect_plan_ex(const b5_scan_t *scan, unsigned range, float depth, bool allow_reverse,
                     B5RectPlan *p) {
    memset(p, 0, sizeof(*p));
    if (!range || range >= B5_RANGE_FAR_MM || !(depth > 0) || depth > 1000) return false;
    p->obstacle_depth_mm = depth;
    p->forward_range_mm = range;
    p->sweep_radius_mm = hypotf(B5R_HALF_WIDTH_MM, B5R_PIVOT_FRONT_MM);
    if (B5R_PIVOT_REAR_MM > B5R_PIVOT_FRONT_MM)
        p->sweep_radius_mm = hypotf(B5R_HALF_WIDTH_MM, B5R_PIVOT_REAR_MM);
    float reach = p->sweep_radius_mm + B5R_MARGIN_MM;
    float need = point_need(range, 0, reach);
    unsigned n = scan ? scan->count : 0;
    for (unsigned i = 0; i < n; ++i) {
        const b5_sample_t *s = &scan->sample[i];
        if (!s->range_mm || s->range_mm >= B5_RANGE_FAR_MM) continue;
        float v = point_need(s->range_mm, s->angle_dd, reach);
        if (v > need) need = v;
    }
    p->reverse_needed_mm = need > 0 ? ceilf(need) : 0;
    p->backoff_mm = allow_reverse ? p->reverse_needed_mm : 0;
    if (p->backoff_mm > B5R_BACKOFF_MAX_MM) return false;
    p->first_turn_clear_mm = point_clear(range, 0, p->backoff_mm, p->sweep_radius_mm);
    for (unsigned i = 0; i < n; ++i) {
        const b5_sample_t *s = &scan->sample[i];
        if (!s->range_mm || s->range_mm >= B5_RANGE_FAR_MM) continue;
        float c = point_clear(s->range_mm, s->angle_dd, p->backoff_mm, p->sweep_radius_mm);
        if (c < p->first_turn_clear_mm) p->first_turn_clear_mm = c;
    }
    p->turn_clearance_ok = p->first_turn_clear_mm >= B5R_MARGIN_MM;
    p->near_y_mm = B5R_SONAR_AHEAD_MM + (float)range - B5R_RANGE_ALLOW_MM;
    p->far_y_mm = B5R_SONAR_AHEAD_MM + (float)range + B5R_RANGE_ALLOW_MM + depth;
    /* Pass position: rear past the far face; after the line-ward turn the
     * near body side past it; and the final in-place restore turn ABOVE the
     * obstacle swings the rear corners (radius hypot(half width, rear))
     * straight down towards it. The largest extent plus MARGIN governs. */
    float extent = B5R_PIVOT_REAR_MM > B5R_HALF_WIDTH_MM ? B5R_PIVOT_REAR_MM : B5R_HALF_WIDTH_MM;
    p->rear_corner_radius_mm = hypotf(B5R_HALF_WIDTH_MM, B5R_PIVOT_REAR_MM);
    if (p->rear_corner_radius_mm > extent) extent = p->rear_corner_radius_mm;
    p->pass_y_mm = p->far_y_mm + extent + B5R_MARGIN_MM;
    p->rear_past_far_mm = p->pass_y_mm - B5R_PIVOT_REAR_MM - p->far_y_mm;
    p->side_past_far_mm = p->pass_y_mm - B5R_HALF_WIDTH_MM - p->far_y_mm;
    p->side_mm = B5R_SIDE_MM;
    p->forward_mm = ceilf(p->pass_y_mm + p->backoff_mm);
    p->max_offset_mm = p->side_mm - B5R_HALF_WIDTH_MM - B5R_OBST_WIDTH_MM * 0.5f - B5R_MARGIN_MM;
    p->return_expect_mm = p->side_mm - B5R_LINE_AHEAD_MM;
    p->return_search_mm = p->return_expect_mm + B5R_RETURN_WINDOW_MM;
    p->return_max_mm = B5R_RETURN_MAX_MM;
    return p->forward_mm <= 1000.0f && p->side_mm <= 1000.0f;
}

void b5_cross_init(B5LineCross *c, float search_mm) {
    memset(c, 0, sizeof(*c));
    c->lo[0] = c->lo[1] = 1000;
    c->search_mm = search_mm;
}
static void edge(B5LineEdge *e, float p, int v, int base) {
    int rise = base + B5L_CONTRAST > B5L_ABS_LEVEL ? base + B5L_CONTRAST : B5L_ABS_LEVEL;
    if (e->phase == 0) {
        if (v >= rise) {
            if (!e->count++) e->first = p;
            if (e->count >= B5L_CONFIRM_N) { e->phase = 1; e->rise = e->first; e->count = 0; }
        } else e->count = 0;
    } else if (e->phase == 1) {
        if (v <= base + B5L_FALL_MARGIN) {
            if (!e->count++) e->first = p;
            if (e->count >= B5L_CONFIRM_N) {
                float width = e->first - e->rise;
                e->fall = e->first;
                e->phase = width < B5L_WIDTH_MIN_MM || width > B5L_WIDTH_MAX_MM ? 3 : 2;
            }
        } else {
            e->count = 0;
            if (p - e->rise > B5L_WIDTH_MAX_MM) e->phase = 3;
        }
    }
}
unsigned b5_cross_step(B5LineCross *c, float p, uint16_t left, uint16_t right) {
    if (c->result != B5L_PENDING) return c->result;
    uint16_t v[2] = {left > 1000 ? 1000 : left, right > 1000 ? 1000 : right};
    if (!c->based) {
        if (p < B5L_BASE_MM) {
            for (unsigned w = 0; w < 2; ++w) {
                c->sum[w] += v[w];
                if (v[w] < c->lo[w]) c->lo[w] = v[w];
                if (v[w] > c->hi[w]) c->hi[w] = v[w];
            }
            ++c->n;
            return B5L_PENDING;
        }
        if (c->n < B5L_BASE_SAMPLES) return c->result = B5L_NO_SAMPLES;
        for (unsigned w = 0; w < 2; ++w) {
            c->base[w] = (uint16_t)lroundf(c->sum[w] / (float)c->n);
            if (c->hi[w] - c->lo[w] > B5L_BASE_SPREAD) return c->result = B5L_UNSTABLE;
            if (c->base[w] + B5L_CONTRAST > 1000) return c->result = B5L_LOW_CONTRAST;
        }
        c->based = true;
    }
    for (unsigned w = 0; w < 2; ++w) edge(&c->s[w], p, v[w], c->base[w]);
    if (c->s[0].phase == 3 || c->s[1].phase == 3) return c->result = B5L_EDGE;
    if (c->s[0].phase == 2 && c->s[1].phase == 2) {
        float a = (c->s[0].rise + c->s[0].fall) * .5f, b = (c->s[1].rise + c->s[1].fall) * .5f;
        if (fabsf(a - b) > B5L_SKEW_MM) return c->result = B5L_MISMATCH;
        c->centre = (a + b) * .5f;
        return c->result = B5L_FOUND;
    }
    for (unsigned w = 0; w < 2; ++w)
        if (c->s[w].phase == 2 && c->s[!w].phase == 0 &&
            p - c->s[w].rise > B5L_SKEW_MM + B5L_WIDTH_MAX_MM)
            return c->result = B5L_MISMATCH;
    if (c->s[0].phase == 0 && c->s[1].phase == 0 && p > c->search_mm)
        return c->result = B5L_NO_RISE;
    return B5L_PENDING;
}
bool b5_cross_evidence(const B5LineCross *c, uint16_t left, uint16_t right) {
    if (!c->based) return false;
    uint16_t v[2] = {left, right};
    for (unsigned w = 0; w < 2; ++w) {
        int rise = c->base[w] + B5L_CONTRAST > B5L_ABS_LEVEL ? c->base[w] + B5L_CONTRAST : B5L_ABS_LEVEL;
        if (v[w] >= rise) return true;
    }
    return false;
}
const char *b5_cross_name(unsigned r) {
    static const char *const names[] = {"PENDING", "FOUND", "NO_RISE", "EDGE_NOT_STRIPE",
        "UNSTABLE_BACKGROUND", "LOW_CONTRAST", "SENSOR_MISMATCH", "NO_SAMPLES"};
    return r < sizeof(names) / sizeof(names[0]) ? names[r] : "INVALID";
}
