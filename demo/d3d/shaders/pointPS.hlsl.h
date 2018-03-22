#if 0
//
// Generated by Microsoft (R) HLSL Shader Compiler 10.1
//
//
// Buffer Definitions: 
//
// cbuffer constBuf
// {
//
//   struct PointShaderConst
//   {
//       
//       float4x4 modelView;            // Offset:    0
//       float4x4 projection;           // Offset:   64
//       float4x4 lightTransform;       // Offset:  128
//       float4 colors[8];              // Offset:  192
//       float4 shadowTaps[12];         // Offset:  320
//       float3 lightPos;               // Offset:  512
//       float _pad0;                   // Offset:  524
//       float3 lightDir;               // Offset:  528
//       float _pad1;                   // Offset:  540
//       float pointRadius;             // Offset:  544
//       float pointScale;              // Offset:  548
//       float spotMin;                 // Offset:  552
//       float spotMax;                 // Offset:  556
//       int mode;                      // Offset:  560
//       int _pad2[3];                  // Offset:  576
//
//   } gParams;                         // Offset:    0 Size:   612
//
// }
//
//
// Resource Bindings:
//
// Name                                 Type  Format         Dim      HLSL Bind  Count
// ------------------------------ ---------- ------- ----------- -------------- ------
// shadowSampler                   sampler_c      NA          NA             s0      1 
// shadowTexture                     texture   float          2d             t0      1 
// constBuf                          cbuffer      NA          NA            cb0      1 
//
//
//
// Input signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// SV_POSITION              0   xyzw        0      POS   float       
// TEXCOORD                 0   xy          1     NONE   float   xy  
// TEXCOORD                 1   xyzw        2     NONE   float   xyzw
// TEXCOORD                 2   xyz         3     NONE   float   xyz 
// TEXCOORD                 3   xyzw        4     NONE   float   xyzw
// TEXCOORD                 4   xyz         5     NONE   float       
// TEXCOORD                 5   xyz         6     NONE   float       
//
//
// Output signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// SV_TARGET                0   xyzw        0   TARGET   float   xyzw
//
ps_5_0
dcl_globalFlags refactoringAllowed
dcl_constantbuffer CB0[36], immediateIndexed
dcl_sampler s0, mode_comparison
dcl_resource_texture2d (float,float,float,float) t0
dcl_input_ps linear v1.xy
dcl_input_ps linear v2.xyzw
dcl_input_ps linear v3.xyz
dcl_input_ps linear v4.xyzw
dcl_output o0.xyzw
dcl_temps 4
mad r0.xy, v1.xyxx, l(2.000000, -2.000000, 0.000000, 0.000000), l(-1.000000, 1.000000, 0.000000, 0.000000)
dp2 r0.x, r0.xyxx, r0.xyxx
lt r0.y, l(1.000000), r0.x
discard_nz r0.y
add r0.x, -r0.x, l(1.000000)
sqrt r0.z, r0.x
ieq r0.w, cb0[35].x, l(2)
if_nz r0.w
  mul r0.w, r0.z, v4.w
  mul o0.xyz, r0.wwww, v4.xyzx
  mov o0.w, r0.w
  ret 
endif 
div r1.xyz, v2.xyzx, v2.wwww
mad r2.xyz, r1.xyzx, l(0.500000, 0.500000, 1.000000, 0.000000), l(0.500000, 0.500000, 0.000000, 0.000000)
lt r0.w, r2.x, l(0.000000)
lt r1.z, l(1.000000), r2.x
or r0.w, r0.w, r1.z
if_z r0.w
  lt r0.w, r2.y, l(0.000000)
  lt r1.z, l(1.000000), r2.y
  or r0.w, r0.w, r1.z
  if_z r0.w
    add r0.w, -cb0[20].y, l(1.000000)
    mul r3.x, cb0[20].x, l(0.002000)
    mul r3.y, r0.w, l(0.002000)
    add r2.w, -r2.y, l(1.000000)
    add r1.zw, r2.xxxw, r3.xxxy
    sample_c_indexable(texture2d)(float,float,float,float) r0.w, r1.zwzz, t0.xxxx, s0, r2.z
    add r1.z, -cb0[21].y, l(1.000000)
    mul r3.x, cb0[21].x, l(0.002000)
    mul r3.y, r1.z, l(0.002000)
    add r1.zw, r2.xxxw, r3.xxxy
    sample_c_indexable(texture2d)(float,float,float,float) r1.z, r1.zwzz, t0.xxxx, s0, r2.z
    add r0.w, r0.w, r1.z
    add r1.z, -cb0[22].y, l(1.000000)
    mul r3.x, cb0[22].x, l(0.002000)
    mul r3.y, r1.z, l(0.002000)
    add r1.zw, r2.xxxw, r3.xxxy
    sample_c_indexable(texture2d)(float,float,float,float) r1.z, r1.zwzz, t0.xxxx, s0, r2.z
    add r0.w, r0.w, r1.z
    add r1.z, -cb0[23].y, l(1.000000)
    mul r3.x, cb0[23].x, l(0.002000)
    mul r3.y, r1.z, l(0.002000)
    add r1.zw, r2.xxxw, r3.xxxy
    sample_c_indexable(texture2d)(float,float,float,float) r1.z, r1.zwzz, t0.xxxx, s0, r2.z
    add r0.w, r0.w, r1.z
    add r1.z, -cb0[24].y, l(1.000000)
    mul r3.x, cb0[24].x, l(0.002000)
    mul r3.y, r1.z, l(0.002000)
    add r1.zw, r2.xxxw, r3.xxxy
    sample_c_indexable(texture2d)(float,float,float,float) r1.z, r1.zwzz, t0.xxxx, s0, r2.z
    add r0.w, r0.w, r1.z
    add r1.z, -cb0[25].y, l(1.000000)
    mul r3.x, cb0[25].x, l(0.002000)
    mul r3.y, r1.z, l(0.002000)
    add r1.zw, r2.xxxw, r3.xxxy
    sample_c_indexable(texture2d)(float,float,float,float) r1.z, r1.zwzz, t0.xxxx, s0, r2.z
    add r0.w, r0.w, r1.z
    add r1.z, -cb0[26].y, l(1.000000)
    mul r3.x, cb0[26].x, l(0.002000)
    mul r3.y, r1.z, l(0.002000)
    add r1.zw, r2.xxxw, r3.xxxy
    sample_c_indexable(texture2d)(float,float,float,float) r1.z, r1.zwzz, t0.xxxx, s0, r2.z
    add r0.w, r0.w, r1.z
    add r1.z, -cb0[27].y, l(1.000000)
    mul r3.x, cb0[27].x, l(0.002000)
    mul r3.y, r1.z, l(0.002000)
    add r1.zw, r2.xxxw, r3.xxxy
    sample_c_indexable(texture2d)(float,float,float,float) r1.z, r1.zwzz, t0.xxxx, s0, r2.z
    add r0.w, r0.w, r1.z
    mul r0.w, r0.w, l(0.125000)
  else 
    mov r0.w, l(1.000000)
  endif 
else 
  mov r0.w, l(1.000000)
endif 
dp2 r1.x, r1.xyxx, r1.xyxx
add r1.y, -cb0[34].w, cb0[34].z
add r1.x, r1.x, -cb0[34].w
div r1.y, l(1.000000, 1.000000, 1.000000, 1.000000), r1.y
mul_sat r1.x, r1.y, r1.x
mad r1.y, r1.x, l(-2.000000), l(3.000000)
mul r1.x, r1.x, r1.x
mul r1.x, r1.x, r1.y
max r1.x, r1.x, l(0.050000)
mul r1.yzw, v4.xxyz, l(0.000000, 0.900000, 0.900000, 0.900000)
mad r0.xy, v1.xyxx, l(2.000000, -2.000000, 0.000000, 0.000000), l(-1.000000, 1.000000, 0.000000, 0.000000)
dp3 r0.x, v3.xyzx, r0.xyzx
mad r0.x, r0.x, l(-0.500000), l(0.500000)
mul r0.x, r0.x, r0.x
mul r0.xyz, r0.xxxx, r1.yzwy
max r0.w, r0.w, l(0.200000)
mul r0.xyz, r0.wwww, r0.xyzx
mul r0.xyz, r1.xxxx, r0.xyzx
log r0.xyz, |r0.xyzx|
mul r0.xyz, r0.xyzx, l(0.454545, 0.454545, 0.454545, 0.000000)
exp o0.xyz, r0.xyzx
mov o0.w, l(1.000000)
ret 
// Approximately 101 instruction slots used
#endif

const BYTE g_pointPS[] =
{
     68,  88,  66,  67, 173, 169, 
    190,  22, 152, 187, 123,  29, 
      5, 187,  32,  68, 204,  15, 
    227,  27,   1,   0,   0,   0, 
    232,  17,   0,   0,   5,   0, 
      0,   0,  52,   0,   0,   0, 
      0,   4,   0,   0, 208,   4, 
      0,   0,   4,   5,   0,   0, 
     76,  17,   0,   0,  82,  68, 
     69,  70, 196,   3,   0,   0, 
      1,   0,   0,   0, 196,   0, 
      0,   0,   3,   0,   0,   0, 
     60,   0,   0,   0,   0,   5, 
    255, 255,   0,   1,   0,   0, 
    156,   3,   0,   0,  82,  68, 
     49,  49,  60,   0,   0,   0, 
     24,   0,   0,   0,  32,   0, 
      0,   0,  40,   0,   0,   0, 
     36,   0,   0,   0,  12,   0, 
      0,   0,   0,   0,   0,   0, 
    156,   0,   0,   0,   3,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      1,   0,   0,   0,   3,   0, 
      0,   0, 170,   0,   0,   0, 
      2,   0,   0,   0,   5,   0, 
      0,   0,   4,   0,   0,   0, 
    255, 255, 255, 255,   0,   0, 
      0,   0,   1,   0,   0,   0, 
      1,   0,   0,   0, 184,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   1,   0, 
      0,   0,   1,   0,   0,   0, 
    115, 104,  97, 100, 111, 119, 
     83,  97, 109, 112, 108, 101, 
    114,   0, 115, 104,  97, 100, 
    111, 119,  84, 101, 120, 116, 
    117, 114, 101,   0,  99, 111, 
    110, 115, 116,  66, 117, 102, 
      0, 171, 171, 171, 184,   0, 
      0,   0,   1,   0,   0,   0, 
    220,   0,   0,   0, 112,   2, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   4,   1, 
      0,   0,   0,   0,   0,   0, 
    100,   2,   0,   0,   2,   0, 
      0,   0, 120,   3,   0,   0, 
      0,   0,   0,   0, 255, 255, 
    255, 255,   0,   0,   0,   0, 
    255, 255, 255, 255,   0,   0, 
      0,   0, 103,  80,  97, 114, 
     97, 109, 115,   0,  80, 111, 
    105, 110, 116,  83, 104,  97, 
    100, 101, 114,  67, 111, 110, 
    115, 116,   0, 109, 111, 100, 
    101, 108,  86, 105, 101, 119, 
      0, 102, 108, 111,  97, 116, 
     52, 120,  52,   0,   3,   0, 
      3,   0,   4,   0,   4,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
     39,   1,   0,   0, 112, 114, 
    111, 106, 101,  99, 116, 105, 
    111, 110,   0, 108, 105, 103, 
    104, 116,  84, 114,  97, 110, 
    115, 102, 111, 114, 109,   0, 
     99, 111, 108, 111, 114, 115, 
      0, 102, 108, 111,  97, 116, 
     52,   0,   1,   0,   3,   0, 
      1,   0,   4,   0,   8,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 117,   1, 
      0,   0, 115, 104,  97, 100, 
    111, 119,  84,  97, 112, 115, 
      0, 171,   1,   0,   3,   0, 
      1,   0,   4,   0,  12,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 117,   1, 
      0,   0, 108, 105, 103, 104, 
    116,  80, 111, 115,   0, 102, 
    108, 111,  97, 116,  51,   0, 
      1,   0,   3,   0,   1,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 217,   1,   0,   0, 
     95, 112,  97, 100,  48,   0, 
    102, 108, 111,  97, 116,   0, 
      0,   0,   3,   0,   1,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,  10,   2,   0,   0, 
    108, 105, 103, 104, 116,  68, 
    105, 114,   0,  95, 112,  97, 
    100,  49,   0, 112, 111, 105, 
    110, 116,  82,  97, 100, 105, 
    117, 115,   0, 112, 111, 105, 
    110, 116,  83,  99,  97, 108, 
    101,   0, 115, 112, 111, 116, 
     77, 105, 110,   0, 115, 112, 
    111, 116,  77,  97, 120,   0, 
    109, 111, 100, 101,   0, 105, 
    110, 116,   0, 171,   0,   0, 
      2,   0,   1,   0,   1,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
    111,   2,   0,   0,  95, 112, 
     97, 100,  50,   0, 171, 171, 
      0,   0,   2,   0,   1,   0, 
      1,   0,   3,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 111,   2,   0,   0, 
     29,   1,   0,   0,  48,   1, 
      0,   0,   0,   0,   0,   0, 
     84,   1,   0,   0,  48,   1, 
      0,   0,  64,   0,   0,   0, 
     95,   1,   0,   0,  48,   1, 
      0,   0, 128,   0,   0,   0, 
    110,   1,   0,   0, 124,   1, 
      0,   0, 192,   0,   0,   0, 
    160,   1,   0,   0, 172,   1, 
      0,   0,  64,   1,   0,   0, 
    208,   1,   0,   0, 224,   1, 
      0,   0,   0,   2,   0,   0, 
      4,   2,   0,   0,  16,   2, 
      0,   0,  12,   2,   0,   0, 
     52,   2,   0,   0, 224,   1, 
      0,   0,  16,   2,   0,   0, 
     61,   2,   0,   0,  16,   2, 
      0,   0,  28,   2,   0,   0, 
     67,   2,   0,   0,  16,   2, 
      0,   0,  32,   2,   0,   0, 
     79,   2,   0,   0,  16,   2, 
      0,   0,  36,   2,   0,   0, 
     90,   2,   0,   0,  16,   2, 
      0,   0,  40,   2,   0,   0, 
     98,   2,   0,   0,  16,   2, 
      0,   0,  44,   2,   0,   0, 
    106,   2,   0,   0, 116,   2, 
      0,   0,  48,   2,   0,   0, 
    152,   2,   0,   0, 160,   2, 
      0,   0,  64,   2,   0,   0, 
      5,   0,   0,   0,   1,   0, 
    144,   0,   0,   0,  15,   0, 
    196,   2,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,  12,   1,   0,   0, 
     77, 105,  99, 114, 111, 115, 
    111, 102, 116,  32,  40,  82, 
     41,  32,  72,  76,  83,  76, 
     32,  83, 104,  97, 100, 101, 
    114,  32,  67, 111, 109, 112, 
    105, 108, 101, 114,  32,  49, 
     48,  46,  49,   0,  73,  83, 
     71,  78, 200,   0,   0,   0, 
      7,   0,   0,   0,   8,   0, 
      0,   0, 176,   0,   0,   0, 
      0,   0,   0,   0,   1,   0, 
      0,   0,   3,   0,   0,   0, 
      0,   0,   0,   0,  15,   0, 
      0,   0, 188,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      1,   0,   0,   0,   3,   3, 
      0,   0, 188,   0,   0,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      2,   0,   0,   0,  15,  15, 
      0,   0, 188,   0,   0,   0, 
      2,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      3,   0,   0,   0,   7,   7, 
      0,   0, 188,   0,   0,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      4,   0,   0,   0,  15,  15, 
      0,   0, 188,   0,   0,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      5,   0,   0,   0,   7,   0, 
      0,   0, 188,   0,   0,   0, 
      5,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      6,   0,   0,   0,   7,   0, 
      0,   0,  83,  86,  95,  80, 
     79,  83,  73,  84,  73,  79, 
     78,   0,  84,  69,  88,  67, 
     79,  79,  82,  68,   0, 171, 
    171, 171,  79,  83,  71,  78, 
     44,   0,   0,   0,   1,   0, 
      0,   0,   8,   0,   0,   0, 
     32,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,  15,   0,   0,   0, 
     83,  86,  95,  84,  65,  82, 
     71,  69,  84,   0, 171, 171, 
     83,  72,  69,  88,  64,  12, 
      0,   0,  80,   0,   0,   0, 
     16,   3,   0,   0, 106,   8, 
      0,   1,  89,   0,   0,   4, 
     70, 142,  32,   0,   0,   0, 
      0,   0,  36,   0,   0,   0, 
     90,   8,   0,   3,   0,  96, 
     16,   0,   0,   0,   0,   0, 
     88,  24,   0,   4,   0, 112, 
     16,   0,   0,   0,   0,   0, 
     85,  85,   0,   0,  98,  16, 
      0,   3,  50,  16,  16,   0, 
      1,   0,   0,   0,  98,  16, 
      0,   3, 242,  16,  16,   0, 
      2,   0,   0,   0,  98,  16, 
      0,   3, 114,  16,  16,   0, 
      3,   0,   0,   0,  98,  16, 
      0,   3, 242,  16,  16,   0, 
      4,   0,   0,   0, 101,   0, 
      0,   3, 242,  32,  16,   0, 
      0,   0,   0,   0, 104,   0, 
      0,   2,   4,   0,   0,   0, 
     50,   0,   0,  15,  50,   0, 
     16,   0,   0,   0,   0,   0, 
     70,  16,  16,   0,   1,   0, 
      0,   0,   2,  64,   0,   0, 
      0,   0,   0,  64,   0,   0, 
      0, 192,   0,   0,   0,   0, 
      0,   0,   0,   0,   2,  64, 
      0,   0,   0,   0, 128, 191, 
      0,   0, 128,  63,   0,   0, 
      0,   0,   0,   0,   0,   0, 
     15,   0,   0,   7,  18,   0, 
     16,   0,   0,   0,   0,   0, 
     70,   0,  16,   0,   0,   0, 
      0,   0,  70,   0,  16,   0, 
      0,   0,   0,   0,  49,   0, 
      0,   7,  34,   0,  16,   0, 
      0,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     10,   0,  16,   0,   0,   0, 
      0,   0,  13,   0,   4,   3, 
     26,   0,  16,   0,   0,   0, 
      0,   0,   0,   0,   0,   8, 
     18,   0,  16,   0,   0,   0, 
      0,   0,  10,   0,  16, 128, 
     65,   0,   0,   0,   0,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  75,   0, 
      0,   5,  66,   0,  16,   0, 
      0,   0,   0,   0,  10,   0, 
     16,   0,   0,   0,   0,   0, 
     32,   0,   0,   8, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     10, 128,  32,   0,   0,   0, 
      0,   0,  35,   0,   0,   0, 
      1,  64,   0,   0,   2,   0, 
      0,   0,  31,   0,   4,   3, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  56,   0,   0,   7, 
    130,   0,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      0,   0,   0,   0,  58,  16, 
     16,   0,   4,   0,   0,   0, 
     56,   0,   0,   7, 114,  32, 
     16,   0,   0,   0,   0,   0, 
    246,  15,  16,   0,   0,   0, 
      0,   0,  70,  18,  16,   0, 
      4,   0,   0,   0,  54,   0, 
      0,   5, 130,  32,  16,   0, 
      0,   0,   0,   0,  58,   0, 
     16,   0,   0,   0,   0,   0, 
     62,   0,   0,   1,  21,   0, 
      0,   1,  14,   0,   0,   7, 
    114,   0,  16,   0,   1,   0, 
      0,   0,  70,  18,  16,   0, 
      2,   0,   0,   0, 246,  31, 
     16,   0,   2,   0,   0,   0, 
     50,   0,   0,  15, 114,   0, 
     16,   0,   2,   0,   0,   0, 
     70,   2,  16,   0,   1,   0, 
      0,   0,   2,  64,   0,   0, 
      0,   0,   0,  63,   0,   0, 
      0,  63,   0,   0, 128,  63, 
      0,   0,   0,   0,   2,  64, 
      0,   0,   0,   0,   0,  63, 
      0,   0,   0,  63,   0,   0, 
      0,   0,   0,   0,   0,   0, 
     49,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     10,   0,  16,   0,   2,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0,   0,   0,  49,   0, 
      0,   7,  66,   0,  16,   0, 
      1,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     10,   0,  16,   0,   2,   0, 
      0,   0,  60,   0,   0,   7, 
    130,   0,  16,   0,   0,   0, 
      0,   0,  58,   0,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   1,   0,   0,   0, 
     31,   0,   0,   3,  58,   0, 
     16,   0,   0,   0,   0,   0, 
     49,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     26,   0,  16,   0,   2,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0,   0,   0,  49,   0, 
      0,   7,  66,   0,  16,   0, 
      1,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     26,   0,  16,   0,   2,   0, 
      0,   0,  60,   0,   0,   7, 
    130,   0,  16,   0,   0,   0, 
      0,   0,  58,   0,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   1,   0,   0,   0, 
     31,   0,   0,   3,  58,   0, 
     16,   0,   0,   0,   0,   0, 
      0,   0,   0,   9, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     26, 128,  32, 128,  65,   0, 
      0,   0,   0,   0,   0,   0, 
     20,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     56,   0,   0,   8,  18,   0, 
     16,   0,   3,   0,   0,   0, 
     10, 128,  32,   0,   0,   0, 
      0,   0,  20,   0,   0,   0, 
      1,  64,   0,   0, 111,  18, 
      3,  59,  56,   0,   0,   7, 
     34,   0,  16,   0,   3,   0, 
      0,   0,  58,   0,  16,   0, 
      0,   0,   0,   0,   1,  64, 
      0,   0, 111,  18,   3,  59, 
      0,   0,   0,   8, 130,   0, 
     16,   0,   2,   0,   0,   0, 
     26,   0,  16, 128,  65,   0, 
      0,   0,   2,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
    128,  63,   0,   0,   0,   7, 
    194,   0,  16,   0,   1,   0, 
      0,   0,   6,  12,  16,   0, 
      2,   0,   0,   0,   6,   4, 
     16,   0,   3,   0,   0,   0, 
     70,   0,   0, 141, 194,   0, 
      0, 128,  67,  85,  21,   0, 
    130,   0,  16,   0,   0,   0, 
      0,   0, 230,  10,  16,   0, 
      1,   0,   0,   0,   6, 112, 
     16,   0,   0,   0,   0,   0, 
      0,  96,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      2,   0,   0,   0,   0,   0, 
      0,   9,  66,   0,  16,   0, 
      1,   0,   0,   0,  26, 128, 
     32, 128,  65,   0,   0,   0, 
      0,   0,   0,   0,  21,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  56,   0, 
      0,   8,  18,   0,  16,   0, 
      3,   0,   0,   0,  10, 128, 
     32,   0,   0,   0,   0,   0, 
     21,   0,   0,   0,   1,  64, 
      0,   0, 111,  18,   3,  59, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
    111,  18,   3,  59,   0,   0, 
      0,   7, 194,   0,  16,   0, 
      1,   0,   0,   0,   6,  12, 
     16,   0,   2,   0,   0,   0, 
      6,   4,  16,   0,   3,   0, 
      0,   0,  70,   0,   0, 141, 
    194,   0,   0, 128,  67,  85, 
     21,   0,  66,   0,  16,   0, 
      1,   0,   0,   0, 230,  10, 
     16,   0,   1,   0,   0,   0, 
      6, 112,  16,   0,   0,   0, 
      0,   0,   0,  96,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   2,   0,   0,   0, 
      0,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   9,  66,   0,  16,   0, 
      1,   0,   0,   0,  26, 128, 
     32, 128,  65,   0,   0,   0, 
      0,   0,   0,   0,  22,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  56,   0, 
      0,   8,  18,   0,  16,   0, 
      3,   0,   0,   0,  10, 128, 
     32,   0,   0,   0,   0,   0, 
     22,   0,   0,   0,   1,  64, 
      0,   0, 111,  18,   3,  59, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
    111,  18,   3,  59,   0,   0, 
      0,   7, 194,   0,  16,   0, 
      1,   0,   0,   0,   6,  12, 
     16,   0,   2,   0,   0,   0, 
      6,   4,  16,   0,   3,   0, 
      0,   0,  70,   0,   0, 141, 
    194,   0,   0, 128,  67,  85, 
     21,   0,  66,   0,  16,   0, 
      1,   0,   0,   0, 230,  10, 
     16,   0,   1,   0,   0,   0, 
      6, 112,  16,   0,   0,   0, 
      0,   0,   0,  96,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   2,   0,   0,   0, 
      0,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   9,  66,   0,  16,   0, 
      1,   0,   0,   0,  26, 128, 
     32, 128,  65,   0,   0,   0, 
      0,   0,   0,   0,  23,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  56,   0, 
      0,   8,  18,   0,  16,   0, 
      3,   0,   0,   0,  10, 128, 
     32,   0,   0,   0,   0,   0, 
     23,   0,   0,   0,   1,  64, 
      0,   0, 111,  18,   3,  59, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
    111,  18,   3,  59,   0,   0, 
      0,   7, 194,   0,  16,   0, 
      1,   0,   0,   0,   6,  12, 
     16,   0,   2,   0,   0,   0, 
      6,   4,  16,   0,   3,   0, 
      0,   0,  70,   0,   0, 141, 
    194,   0,   0, 128,  67,  85, 
     21,   0,  66,   0,  16,   0, 
      1,   0,   0,   0, 230,  10, 
     16,   0,   1,   0,   0,   0, 
      6, 112,  16,   0,   0,   0, 
      0,   0,   0,  96,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   2,   0,   0,   0, 
      0,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   9,  66,   0,  16,   0, 
      1,   0,   0,   0,  26, 128, 
     32, 128,  65,   0,   0,   0, 
      0,   0,   0,   0,  24,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  56,   0, 
      0,   8,  18,   0,  16,   0, 
      3,   0,   0,   0,  10, 128, 
     32,   0,   0,   0,   0,   0, 
     24,   0,   0,   0,   1,  64, 
      0,   0, 111,  18,   3,  59, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
    111,  18,   3,  59,   0,   0, 
      0,   7, 194,   0,  16,   0, 
      1,   0,   0,   0,   6,  12, 
     16,   0,   2,   0,   0,   0, 
      6,   4,  16,   0,   3,   0, 
      0,   0,  70,   0,   0, 141, 
    194,   0,   0, 128,  67,  85, 
     21,   0,  66,   0,  16,   0, 
      1,   0,   0,   0, 230,  10, 
     16,   0,   1,   0,   0,   0, 
      6, 112,  16,   0,   0,   0, 
      0,   0,   0,  96,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   2,   0,   0,   0, 
      0,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   9,  66,   0,  16,   0, 
      1,   0,   0,   0,  26, 128, 
     32, 128,  65,   0,   0,   0, 
      0,   0,   0,   0,  25,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  56,   0, 
      0,   8,  18,   0,  16,   0, 
      3,   0,   0,   0,  10, 128, 
     32,   0,   0,   0,   0,   0, 
     25,   0,   0,   0,   1,  64, 
      0,   0, 111,  18,   3,  59, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
    111,  18,   3,  59,   0,   0, 
      0,   7, 194,   0,  16,   0, 
      1,   0,   0,   0,   6,  12, 
     16,   0,   2,   0,   0,   0, 
      6,   4,  16,   0,   3,   0, 
      0,   0,  70,   0,   0, 141, 
    194,   0,   0, 128,  67,  85, 
     21,   0,  66,   0,  16,   0, 
      1,   0,   0,   0, 230,  10, 
     16,   0,   1,   0,   0,   0, 
      6, 112,  16,   0,   0,   0, 
      0,   0,   0,  96,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   2,   0,   0,   0, 
      0,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   9,  66,   0,  16,   0, 
      1,   0,   0,   0,  26, 128, 
     32, 128,  65,   0,   0,   0, 
      0,   0,   0,   0,  26,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  56,   0, 
      0,   8,  18,   0,  16,   0, 
      3,   0,   0,   0,  10, 128, 
     32,   0,   0,   0,   0,   0, 
     26,   0,   0,   0,   1,  64, 
      0,   0, 111,  18,   3,  59, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
    111,  18,   3,  59,   0,   0, 
      0,   7, 194,   0,  16,   0, 
      1,   0,   0,   0,   6,  12, 
     16,   0,   2,   0,   0,   0, 
      6,   4,  16,   0,   3,   0, 
      0,   0,  70,   0,   0, 141, 
    194,   0,   0, 128,  67,  85, 
     21,   0,  66,   0,  16,   0, 
      1,   0,   0,   0, 230,  10, 
     16,   0,   1,   0,   0,   0, 
      6, 112,  16,   0,   0,   0, 
      0,   0,   0,  96,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   2,   0,   0,   0, 
      0,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   9,  66,   0,  16,   0, 
      1,   0,   0,   0,  26, 128, 
     32, 128,  65,   0,   0,   0, 
      0,   0,   0,   0,  27,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  63,  56,   0, 
      0,   8,  18,   0,  16,   0, 
      3,   0,   0,   0,  10, 128, 
     32,   0,   0,   0,   0,   0, 
     27,   0,   0,   0,   1,  64, 
      0,   0, 111,  18,   3,  59, 
     56,   0,   0,   7,  34,   0, 
     16,   0,   3,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
    111,  18,   3,  59,   0,   0, 
      0,   7, 194,   0,  16,   0, 
      1,   0,   0,   0,   6,  12, 
     16,   0,   2,   0,   0,   0, 
      6,   4,  16,   0,   3,   0, 
      0,   0,  70,   0,   0, 141, 
    194,   0,   0, 128,  67,  85, 
     21,   0,  66,   0,  16,   0, 
      1,   0,   0,   0, 230,  10, 
     16,   0,   1,   0,   0,   0, 
      6, 112,  16,   0,   0,   0, 
      0,   0,   0,  96,  16,   0, 
      0,   0,   0,   0,  42,   0, 
     16,   0,   2,   0,   0,   0, 
      0,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      1,   0,   0,   0,  56,   0, 
      0,   7, 130,   0,  16,   0, 
      0,   0,   0,   0,  58,   0, 
     16,   0,   0,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
      0,  62,  18,   0,   0,   1, 
     54,   0,   0,   5, 130,   0, 
     16,   0,   0,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
    128,  63,  21,   0,   0,   1, 
     18,   0,   0,   1,  54,   0, 
      0,   5, 130,   0,  16,   0, 
      0,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     21,   0,   0,   1,  15,   0, 
      0,   7,  18,   0,  16,   0, 
      1,   0,   0,   0,  70,   0, 
     16,   0,   1,   0,   0,   0, 
     70,   0,  16,   0,   1,   0, 
      0,   0,   0,   0,   0,  10, 
     34,   0,  16,   0,   1,   0, 
      0,   0,  58, 128,  32, 128, 
     65,   0,   0,   0,   0,   0, 
      0,   0,  34,   0,   0,   0, 
     42, 128,  32,   0,   0,   0, 
      0,   0,  34,   0,   0,   0, 
      0,   0,   0,   9,  18,   0, 
     16,   0,   1,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,  58, 128,  32, 128, 
     65,   0,   0,   0,   0,   0, 
      0,   0,  34,   0,   0,   0, 
     14,   0,   0,  10,  34,   0, 
     16,   0,   1,   0,   0,   0, 
      2,  64,   0,   0,   0,   0, 
    128,  63,   0,   0, 128,  63, 
      0,   0, 128,  63,   0,   0, 
    128,  63,  26,   0,  16,   0, 
      1,   0,   0,   0,  56,  32, 
      0,   7,  18,   0,  16,   0, 
      1,   0,   0,   0,  26,   0, 
     16,   0,   1,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,  50,   0,   0,   9, 
     34,   0,  16,   0,   1,   0, 
      0,   0,  10,   0,  16,   0, 
      1,   0,   0,   0,   1,  64, 
      0,   0,   0,   0,   0, 192, 
      1,  64,   0,   0,   0,   0, 
     64,  64,  56,   0,   0,   7, 
     18,   0,  16,   0,   1,   0, 
      0,   0,  10,   0,  16,   0, 
      1,   0,   0,   0,  10,   0, 
     16,   0,   1,   0,   0,   0, 
     56,   0,   0,   7,  18,   0, 
     16,   0,   1,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,  26,   0,  16,   0, 
      1,   0,   0,   0,  52,   0, 
      0,   7,  18,   0,  16,   0, 
      1,   0,   0,   0,  10,   0, 
     16,   0,   1,   0,   0,   0, 
      1,  64,   0,   0, 205, 204, 
     76,  61,  56,   0,   0,  10, 
    226,   0,  16,   0,   1,   0, 
      0,   0,   6,  25,  16,   0, 
      4,   0,   0,   0,   2,  64, 
      0,   0,   0,   0,   0,   0, 
    102, 102, 102,  63, 102, 102, 
    102,  63, 102, 102, 102,  63, 
     50,   0,   0,  15,  50,   0, 
     16,   0,   0,   0,   0,   0, 
     70,  16,  16,   0,   1,   0, 
      0,   0,   2,  64,   0,   0, 
      0,   0,   0,  64,   0,   0, 
      0, 192,   0,   0,   0,   0, 
      0,   0,   0,   0,   2,  64, 
      0,   0,   0,   0, 128, 191, 
      0,   0, 128,  63,   0,   0, 
      0,   0,   0,   0,   0,   0, 
     16,   0,   0,   7,  18,   0, 
     16,   0,   0,   0,   0,   0, 
     70,  18,  16,   0,   3,   0, 
      0,   0,  70,   2,  16,   0, 
      0,   0,   0,   0,  50,   0, 
      0,   9,  18,   0,  16,   0, 
      0,   0,   0,   0,  10,   0, 
     16,   0,   0,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
      0, 191,   1,  64,   0,   0, 
      0,   0,   0,  63,  56,   0, 
      0,   7,  18,   0,  16,   0, 
      0,   0,   0,   0,  10,   0, 
     16,   0,   0,   0,   0,   0, 
     10,   0,  16,   0,   0,   0, 
      0,   0,  56,   0,   0,   7, 
    114,   0,  16,   0,   0,   0, 
      0,   0,   6,   0,  16,   0, 
      0,   0,   0,   0, 150,   7, 
     16,   0,   1,   0,   0,   0, 
     52,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     58,   0,  16,   0,   0,   0, 
      0,   0,   1,  64,   0,   0, 
    205, 204,  76,  62,  56,   0, 
      0,   7, 114,   0,  16,   0, 
      0,   0,   0,   0, 246,  15, 
     16,   0,   0,   0,   0,   0, 
     70,   2,  16,   0,   0,   0, 
      0,   0,  56,   0,   0,   7, 
    114,   0,  16,   0,   0,   0, 
      0,   0,   6,   0,  16,   0, 
      1,   0,   0,   0,  70,   2, 
     16,   0,   0,   0,   0,   0, 
     47,   0,   0,   6, 114,   0, 
     16,   0,   0,   0,   0,   0, 
     70,   2,  16, 128, 129,   0, 
      0,   0,   0,   0,   0,   0, 
     56,   0,   0,  10, 114,   0, 
     16,   0,   0,   0,   0,   0, 
     70,   2,  16,   0,   0,   0, 
      0,   0,   2,  64,   0,   0, 
     47, 186, 232,  62,  47, 186, 
    232,  62,  47, 186, 232,  62, 
      0,   0,   0,   0,  25,   0, 
      0,   5, 114,  32,  16,   0, 
      0,   0,   0,   0,  70,   2, 
     16,   0,   0,   0,   0,   0, 
     54,   0,   0,   5, 130,  32, 
     16,   0,   0,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
    128,  63,  62,   0,   0,   1, 
     83,  84,  65,  84, 148,   0, 
      0,   0, 101,   0,   0,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   0,   5,   0,   0,   0, 
     75,   0,   0,   0,   1,   0, 
      0,   0,   2,   0,   0,   0, 
      4,   0,   0,   0,   3,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      8,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0
};
