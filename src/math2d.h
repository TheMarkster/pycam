#ifndef MATH2D_H
#define MATH2D_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vec2d {
    float v[2];
} vec2d;

typedef struct mat2d {
    float m[2][2];
} mat2d;

// Constructors
mat2d mat_from_rvec(vec2d a, vec2d b);
mat2d mat_from_cvec(vec2d a, vec2d b);

vec2d cvec1_from_mat(mat2d m);
vec2d cvec2_from_mat(mat2d m);
vec2d rvec1_from_mat(mat2d m);
vec2d rvec2_from_mat(mat2d m);

vec2d vec_from_float(float x, float y);

//Operations
// Muliplication
mat2d mm_multiply(mat2d a, mat2d b);
vec2d mv_multiply(mat2d m, vec2d v);
vec2d vm_multiply(vec2d v, mat2d m);

mat2d mat_smul(mat2d a, float b);
mat2d mat_sdiv(mat2d a, float b);
mat2d mat_sadd(mat2d a, float b);
mat2d mat_ssub(mat2d a, float b);

vec2d vec_add(vec2d a, vec2d b);
vec2d vec_sub(vec2d a, vec2d b);

vec2d vec_sadd(vec2d a, float b);
vec2d vec_ssub(vec2d a, float b);
vec2d vec_smul(vec2d a, float b);
vec2d vec_sdiv(vec2d a, float b);

mat2d inverse(mat2d m);
float determinant(mat2d m);
float dot(vec2d a, vec2d b);
vec2d solve(mat2d m, vec2d s);

#ifdef __cplusplus
}
#endif

#endif