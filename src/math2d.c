#include "math2d.h"
#include <float.h>


// Constructors
mat2d mat_from_rvec(vec2d a, vec2d b) {
    mat2d m;
    m.m[0][0] = a.v[0];
    m.m[0][1] = a.v[1];
    m.m[1][0] = b.v[0];
    m.m[1][1] = b.v[1];
    return m;
}

mat2d mat_from_cvec(vec2d a, vec2d b) {
    mat2d m;
    m.m[0][0] = a.v[0];
    m.m[0][1] = b.v[0];
    m.m[1][0] = a.v[1];
    m.m[1][1] = b.v[1];
    return m;
}

vec2d cvec1_from_mat(mat2d m) {
    vec2d v;
    v.v[0] = m.m[0][0];
    v.v[1] = m.m[1][0];
    return v;
}

vec2d cvec2_from_mat(mat2d m) {
    vec2d v;
    v.v[0] = m.m[0][1];
    v.v[1] = m.m[1][1];
    return v;
}

vec2d rvec1_from_mat(mat2d m) {
    vec2d v;
    v.v[0] = m.m[0][0];
    v.v[1] = m.m[0][1];
    return v;
}

vec2d rvec2_from_mat(mat2d m) {
    vec2d v;
    v.v[0] = m.m[1][0];
    v.v[1] = m.m[1][1];
    return v;
}

vec2d vec_from_float(float x, float y) {
    vec2d v;
    v.v[0] = x;
    v.v[1] = y;
    return v;
}

//Operations
mat2d mm_multiply(mat2d a, mat2d b) {
    mat2d m;
    m.m[0][0] = a.m[0][0] * b.m[0][0] + a.m[0][1] * b.m[1][0];
    m.m[0][1] = a.m[0][0] * b.m[0][1] + a.m[0][1] * b.m[1][1];
    m.m[1][0] = a.m[1][0] * b.m[0][0] + a.m[1][1] * b.m[1][0];
    m.m[1][1] = a.m[1][0] * b.m[0][1] + a.m[1][1] * b.m[1][1];
    return m;
}

vec2d mv_multiply(mat2d m, vec2d v) {
    vec2d result;
    result.v[0] = m.m[0][0] * v.v[0] + m.m[0][1] * v.v[1];
    result.v[1] = m.m[1][0] * v.v[0] + m.m[1][1] * v.v[1];
    return result;
}

vec2d vm_multiply(vec2d v, mat2d m) {
    vec2d result;
    result.v[0] = v.v[0] * m.m[0][0] + v.v[1] * m.m[1][0];
    result.v[1] = v.v[0] * m.m[0][1] + v.v[1] * m.m[1][1];
    return result;
}

mat2d mat_smul(mat2d a, float b) {
    mat2d result;
    result.m[0][0] = a.m[0][0] * b;
    result.m[0][1] = a.m[0][1] * b;
    result.m[1][0] = a.m[1][0] * b;
    result.m[1][1] = a.m[1][1] * b;
    return result;
}

mat2d mat_sdiv(mat2d a, float b) {
    mat2d result;
    if (b != 0) {
        result.m[0][0] = a.m[0][0] / b;
        result.m[0][1] = a.m[0][1] / b;
        result.m[1][0] = a.m[1][0] / b;
        result.m[1][1] = a.m[1][1] / b;
    } else {
        // Handle division by zero case
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {\
                if (result.m[i][j] < 0)
                    result.m[i][j] = -FLT_MAX;
                else
                    result.m[i][j] = FLT_MAX;
            }
        }
    }
    return result;
}

mat2d mat_sadd(mat2d a, float b) {
    mat2d result;
    result.m[0][0] = a.m[0][0] + b;
    result.m[0][1] = a.m[0][1] + b;
    result.m[1][0] = a.m[1][0] + b;
    result.m[1][1] = a.m[1][1] + b;
    return result;
}

mat2d mat_ssub(mat2d a, float b) {
    mat2d result;
    result.m[0][0] = a.m[0][0] - b;
    result.m[0][1] = a.m[0][1] - b;
    result.m[1][0] = a.m[1][0] - b;
    result.m[1][1] = a.m[1][1] - b;
    return result;
}

vec2d vec_add(vec2d a, vec2d b) {
    vec2d result;
    result.v[0] = a.v[0] + b.v[0];
    result.v[1] = a.v[1] + b.v[1];
    return result;
}

vec2d vec_sub(vec2d a, vec2d b) {
    vec2d result;
    result.v[0] = a.v[0] - b.v[0];
    result.v[1] = a.v[1] - b.v[1];
    return result;
}

vec2d vec_sadd(vec2d a, float b) {
    vec2d result;
    result.v[0] = a.v[0] + b;
    result.v[1] = a.v[1] + b;
    return result;
}

vec2d vec_ssub(vec2d a, float b) {
    vec2d result;
    result.v[0] = a.v[0] - b;
    result.v[1] = a.v[1] - b;
    return result;
}

vec2d vec_smul(vec2d a, float b) {
    vec2d result;
    result.v[0] = a.v[0] * b;
    result.v[1] = a.v[1] * b;
    return result;
}

vec2d vec_sdiv(vec2d a, float b) {
    vec2d result;
    if (b != 0) {
        result.v[0] = a.v[0] / b;
        result.v[1] = a.v[1] / b;
    } else {
        // Handle division by zero case
        if (a.v[0] < 0)
            result.v[0] = -FLT_MAX;
        else
            result.v[0] = FLT_MAX;

        if (a.v[1] < 0)
            result.v[1] = -FLT_MAX;
        else
            result.v[1] = FLT_MAX;
    }
    return result;
}

mat2d inverse(mat2d m) {
    mat2d inv;
    float d = determinant(m);
    
    if (d == 0) {
        // Handle the case of no inverse (singular matrix)
        inv.m[0][0] = 0; inv.m[0][1] = 0;
        inv.m[1][0] = 0; inv.m[1][1] = 0;
    } else {
        inv.m[0][0] = m.m[1][1] / d;
        inv.m[0][1] = -m.m[0][1] / d;
        inv.m[1][0] = -m.m[1][0] / d;
        inv.m[1][1] = m.m[0][0] / d;
    }
    
    return inv;
}

float determinant(mat2d m) {
    return m.m[0][0] * m.m[1][1] - m.m[0][1] * m.m[1][0];
}

float dot(vec2d a, vec2d b) {
    return a.v[0] * b.v[0] + a.v[1] * b.v[1];
}

vec2d solve(mat2d m, vec2d s) {
    float d = determinant(m);
    vec2d result;

    if (d == 0) {
        result.v[0] = 0; // or handle the case of no solution
        result.v[1] = 0;
    }
    else {
        result.v[0] = (m.m[1][1] * s.v[0] - m.m[0][1] * s.v[1]) / d;
        result.v[1] = (m.m[0][0] * s.v[1] - m.m[1][0] * s.v[0]) / d;
    }

    return result;
}