#if 0
//
// Generated by Microsoft (R) HLSL Shader Compiler 6.3.9600.16384
//
//
// Buffer Definitions: 
//
// cbuffer constBuf
// {
//
//   struct FluidShaderConst
//   {
//       
//       float4x4 modelViewProjection;  // Offset:    0
//       float4x4 modelView;            // Offset:   64
//       float4x4 projection;           // Offset:  128
//       float4x4 inverseModelView;     // Offset:  192
//       float4x4 inverseProjection;    // Offset:  256
//       float4 invTexScale;            // Offset:  320
//       float3 invViewport;            // Offset:  336
//       float _pad0;                   // Offset:  348
//       float blurRadiusWorld;         // Offset:  352
//       float blurScale;               // Offset:  356
//       float blurFalloff;             // Offset:  360
//       int debug;                     // Offset:  364
//       float3 lightPos;               // Offset:  368
//       float _pad1;                   // Offset:  380
//       float3 lightDir;               // Offset:  384
//       float _pad2;                   // Offset:  396
//       float4x4 lightTransform;       // Offset:  400
//       float4 color;                  // Offset:  464
//       float4 clipPosToEye;           // Offset:  480
//       float spotMin;                 // Offset:  496
//       float spotMax;                 // Offset:  500
//       float ior;                     // Offset:  504
//       float pointRadius;             // Offset:  508
//       float4 shadowTaps[12];         // Offset:  512
//
//   } gParams;                         // Offset:    0 Size:   704
//
// }
//
//
// Resource Bindings:
//
// Name                                 Type  Format         Dim Slot Elements
// ------------------------------ ---------- ------- ----------- ---- --------
// constBuf                          cbuffer      NA          NA    0        1
//
//
//
// Input signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// SV_POSITION              0   xyzw        0      POS   float   xy  
// TEXCOORD                 0   xyzw        1     NONE   float   xyzw
// TEXCOORD                 1   xyzw        2     NONE   float   xyzw
// TEXCOORD                 2   xyzw        3     NONE   float   xyzw
// TEXCOORD                 3   xyzw        4     NONE   float   xyzw
//
//
// Output signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// SV_TARGET                0   x           0   TARGET   float   x   
// SV_DEPTH                 0    N/A   oDepth    DEPTH   float    YES
//
ps_5_0
dcl_globalFlags refactoringAllowed
dcl_constantbuffer cb0[22], immediateIndexed
dcl_input_ps_siv linear noperspective v0.xy, position
dcl_input_ps linear v1.xyzw
dcl_input_ps linear v2.xyzw
dcl_input_ps linear v3.xyzw
dcl_input_ps linear v4.xyzw
dcl_output o0.x
dcl_output oDepth
dcl_temps 4
dp2 r0.x, v0.xxxx, cb0[21].xxxx
add r0.x, r0.x, l(-1.000000)
mad r0.y, -v0.y, cb0[21].y, l(1.000000)
mad r0.y, r0.y, l(2.000000), l(-1.000000)
mul r0.yzw, r0.yyyy, cb0[17].xxyz
mad r0.xyz, cb0[16].xyzx, r0.xxxx, r0.yzwy
add r0.xyz, r0.xyzx, cb0[19].xyzx
mul r1.xyzw, r0.yyyy, v2.xyzw
mad r1.xyzw, v1.xyzw, r0.xxxx, r1.xyzw
mad r1.xyzw, v3.xyzw, r0.zzzz, r1.xyzw
dp3 r0.w, r1.xyzx, r1.xyzx
dp3 r1.x, r1.xyzx, v4.xyzx
mad r1.x, -r1.w, v4.w, r1.x
dp3 r1.y, v4.xyzx, v4.xyzx
mad r1.y, -v4.w, v4.w, r1.y
add r1.z, r1.x, r1.x
eq r1.w, r0.w, l(0.000000)
eq r2.x, r1.x, l(0.000000)
and r1.w, r1.w, r2.x
mul r2.x, r0.w, r1.y
mul r2.x, r2.x, l(4.000000)
mad r2.x, r1.z, r1.z, -r2.x
lt r2.y, r2.x, l(0.000000)
not r3.y, r2.y
lt r1.x, r1.x, l(0.000000)
movc r1.x, r1.x, l(-1.000000), l(1.000000)
sqrt r2.x, r2.x
mad r1.x, r1.x, r2.x, r1.z
mul r1.x, r1.x, l(-0.500000)
div r0.w, r1.x, r0.w
div r1.x, r1.y, r1.x
lt r1.y, r1.x, r0.w
movc r3.z, r1.y, r1.x, r0.w
mov r3.xw, l(0,0,0,-1)
movc r1.xy, r2.yyyy, r3.xyxx, r3.zwzz
movc r1.xy, r1.wwww, l(0,-1,0,0), r1.xyxx
not r0.w, r1.y
discard_nz r0.w
mul r0.xyz, r0.xyzx, r1.xxxx
mul r0.yw, r0.yyyy, cb0[9].zzzw
mad r0.xy, cb0[8].zwzz, r0.xxxx, r0.ywyy
mad r0.xy, cb0[10].zwzz, r0.zzzz, r0.xyxx
add r0.xy, r0.xyxx, cb0[11].zwzz
div oDepth, r0.x, r0.y
mov o0.x, r0.z
ret 
// Approximately 46 instruction slots used
#endif

const BYTE g_ellipsoidDepthPS[] =
{
     68,  88,  66,  67,  21, 223, 
    248, 253,  98, 204, 115, 211, 
     23, 174,  18,  67, 157, 195, 
     79,  80,   1,   0,   0,   0, 
    252,  11,   0,   0,   5,   0, 
      0,   0,  52,   0,   0,   0, 
    116,   4,   0,   0,  20,   5, 
      0,   0, 104,   5,   0,   0, 
     96,  11,   0,   0,  82,  68, 
     69,  70,  56,   4,   0,   0, 
      1,   0,   0,   0, 104,   0, 
      0,   0,   1,   0,   0,   0, 
     60,   0,   0,   0,   0,   5, 
    255, 255,   0,   1,   0,   0, 
      4,   4,   0,   0,  82,  68, 
     49,  49,  60,   0,   0,   0, 
     24,   0,   0,   0,  32,   0, 
      0,   0,  40,   0,   0,   0, 
     36,   0,   0,   0,  12,   0, 
      0,   0,   0,   0,   0,   0, 
     92,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      1,   0,   0,   0,   1,   0, 
      0,   0,  99, 111, 110, 115, 
    116,  66, 117, 102,   0, 171, 
    171, 171,  92,   0,   0,   0, 
      1,   0,   0,   0, 128,   0, 
      0,   0, 192,   2,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 168,   0,   0,   0, 
      0,   0,   0,   0, 192,   2, 
      0,   0,   2,   0,   0,   0, 
    224,   3,   0,   0,   0,   0, 
      0,   0, 255, 255, 255, 255, 
      0,   0,   0,   0, 255, 255, 
    255, 255,   0,   0,   0,   0, 
    103,  80,  97, 114,  97, 109, 
    115,   0,  70, 108, 117, 105, 
    100,  83, 104,  97, 100, 101, 
    114,  67, 111, 110, 115, 116, 
      0, 109, 111, 100, 101, 108, 
     86, 105, 101, 119,  80, 114, 
    111, 106, 101,  99, 116, 105, 
    111, 110,   0, 102, 108, 111, 
     97, 116,  52, 120,  52,   0, 
    171, 171,   3,   0,   3,   0, 
      4,   0,   4,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 213,   0, 
      0,   0, 109, 111, 100, 101, 
    108,  86, 105, 101, 119,   0, 
    112, 114, 111, 106, 101,  99, 
    116, 105, 111, 110,   0, 105, 
    110, 118, 101, 114, 115, 101, 
     77, 111, 100, 101, 108,  86, 
    105, 101, 119,   0, 105, 110, 
    118, 101, 114, 115, 101,  80, 
    114, 111, 106, 101,  99, 116, 
    105, 111, 110,   0, 105, 110, 
    118,  84, 101, 120,  83,  99, 
     97, 108, 101,   0, 102, 108, 
    111,  97, 116,  52,   0, 171, 
      1,   0,   3,   0,   1,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,  72,   1,   0,   0, 
    105, 110, 118,  86, 105, 101, 
    119, 112, 111, 114, 116,   0, 
    102, 108, 111,  97, 116,  51, 
      0, 171,   1,   0,   3,   0, 
      1,   0,   3,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 128,   1, 
      0,   0,  95, 112,  97, 100, 
     48,   0, 102, 108, 111,  97, 
    116,   0,   0,   0,   3,   0, 
      1,   0,   1,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 178,   1, 
      0,   0,  98, 108, 117, 114, 
     82,  97, 100, 105, 117, 115, 
     87, 111, 114, 108, 100,   0, 
     98, 108, 117, 114,  83,  99, 
     97, 108, 101,   0,  98, 108, 
    117, 114,  70,  97, 108, 108, 
    111, 102, 102,   0, 100, 101, 
     98, 117, 103,   0, 105, 110, 
    116,   0,   0,   0,   2,   0, 
      1,   0,   1,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   8,   2, 
      0,   0, 108, 105, 103, 104, 
    116,  80, 111, 115,   0,  95, 
    112,  97, 100,  49,   0, 108, 
    105, 103, 104, 116,  68, 105, 
    114,   0,  95, 112,  97, 100, 
     50,   0, 108, 105, 103, 104, 
    116,  84, 114,  97, 110, 115, 
    102, 111, 114, 109,   0,  99, 
    111, 108, 111, 114,   0,  99, 
    108, 105, 112,  80, 111, 115, 
     84, 111,  69, 121, 101,   0, 
    115, 112, 111, 116,  77, 105, 
    110,   0, 115, 112, 111, 116, 
     77,  97, 120,   0, 105, 111, 
    114,   0, 112, 111, 105, 110, 
    116,  82,  97, 100, 105, 117, 
    115,   0, 115, 104,  97, 100, 
    111, 119,  84,  97, 112, 115, 
      0, 171,   1,   0,   3,   0, 
      1,   0,   4,   0,  12,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,  72,   1, 
      0,   0, 193,   0,   0,   0, 
    224,   0,   0,   0,   0,   0, 
      0,   0,   4,   1,   0,   0, 
    224,   0,   0,   0,  64,   0, 
      0,   0,  14,   1,   0,   0, 
    224,   0,   0,   0, 128,   0, 
      0,   0,  25,   1,   0,   0, 
    224,   0,   0,   0, 192,   0, 
      0,   0,  42,   1,   0,   0, 
    224,   0,   0,   0,   0,   1, 
      0,   0,  60,   1,   0,   0, 
     80,   1,   0,   0,  64,   1, 
      0,   0, 116,   1,   0,   0, 
    136,   1,   0,   0,  80,   1, 
      0,   0, 172,   1,   0,   0, 
    184,   1,   0,   0,  92,   1, 
      0,   0, 220,   1,   0,   0, 
    184,   1,   0,   0,  96,   1, 
      0,   0, 236,   1,   0,   0, 
    184,   1,   0,   0, 100,   1, 
      0,   0, 246,   1,   0,   0, 
    184,   1,   0,   0, 104,   1, 
      0,   0,   2,   2,   0,   0, 
     12,   2,   0,   0, 108,   1, 
      0,   0,  48,   2,   0,   0, 
    136,   1,   0,   0, 112,   1, 
      0,   0,  57,   2,   0,   0, 
    184,   1,   0,   0, 124,   1, 
      0,   0,  63,   2,   0,   0, 
    136,   1,   0,   0, 128,   1, 
      0,   0,  72,   2,   0,   0, 
    184,   1,   0,   0, 140,   1, 
      0,   0,  78,   2,   0,   0, 
    224,   0,   0,   0, 144,   1, 
      0,   0,  93,   2,   0,   0, 
     80,   1,   0,   0, 208,   1, 
      0,   0,  99,   2,   0,   0, 
     80,   1,   0,   0, 224,   1, 
      0,   0, 112,   2,   0,   0, 
    184,   1,   0,   0, 240,   1, 
      0,   0, 120,   2,   0,   0, 
    184,   1,   0,   0, 244,   1, 
      0,   0, 128,   2,   0,   0, 
    184,   1,   0,   0, 248,   1, 
      0,   0, 132,   2,   0,   0, 
    184,   1,   0,   0, 252,   1, 
      0,   0, 144,   2,   0,   0, 
    156,   2,   0,   0,   0,   2, 
      0,   0,   5,   0,   0,   0, 
      1,   0, 176,   0,   0,   0, 
     24,   0, 192,   2,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 176,   0, 
      0,   0,  77, 105,  99, 114, 
    111, 115, 111, 102, 116,  32, 
     40,  82,  41,  32,  72,  76, 
     83,  76,  32,  83, 104,  97, 
    100, 101, 114,  32,  67, 111, 
    109, 112, 105, 108, 101, 114, 
     32,  54,  46,  51,  46,  57, 
     54,  48,  48,  46,  49,  54, 
     51,  56,  52,   0, 171, 171, 
     73,  83,  71,  78, 152,   0, 
      0,   0,   5,   0,   0,   0, 
      8,   0,   0,   0, 128,   0, 
      0,   0,   0,   0,   0,   0, 
      1,   0,   0,   0,   3,   0, 
      0,   0,   0,   0,   0,   0, 
     15,   3,   0,   0, 140,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,   1,   0,   0,   0, 
     15,  15,   0,   0, 140,   0, 
      0,   0,   1,   0,   0,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,   2,   0,   0,   0, 
     15,  15,   0,   0, 140,   0, 
      0,   0,   2,   0,   0,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,   3,   0,   0,   0, 
     15,  15,   0,   0, 140,   0, 
      0,   0,   3,   0,   0,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,   4,   0,   0,   0, 
     15,  15,   0,   0,  83,  86, 
     95,  80,  79,  83,  73,  84, 
     73,  79,  78,   0,  84,  69, 
     88,  67,  79,  79,  82,  68, 
      0, 171, 171, 171,  79,  83, 
     71,  78,  76,   0,   0,   0, 
      2,   0,   0,   0,   8,   0, 
      0,   0,  56,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
      0,   0,   0,   0,   1,  14, 
      0,   0,  66,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   3,   0,   0,   0, 
    255, 255, 255, 255,   1,  14, 
      0,   0,  83,  86,  95,  84, 
     65,  82,  71,  69,  84,   0, 
     83,  86,  95,  68,  69,  80, 
     84,  72,   0, 171,  83,  72, 
     69,  88, 240,   5,   0,   0, 
     80,   0,   0,   0, 124,   1, 
      0,   0, 106,   8,   0,   1, 
     89,   0,   0,   4,  70, 142, 
     32,   0,   0,   0,   0,   0, 
     22,   0,   0,   0, 100,  32, 
      0,   4,  50,  16,  16,   0, 
      0,   0,   0,   0,   1,   0, 
      0,   0,  98,  16,   0,   3, 
    242,  16,  16,   0,   1,   0, 
      0,   0,  98,  16,   0,   3, 
    242,  16,  16,   0,   2,   0, 
      0,   0,  98,  16,   0,   3, 
    242,  16,  16,   0,   3,   0, 
      0,   0,  98,  16,   0,   3, 
    242,  16,  16,   0,   4,   0, 
      0,   0, 101,   0,   0,   3, 
     18,  32,  16,   0,   0,   0, 
      0,   0, 101,   0,   0,   2, 
      1, 192,   0,   0, 104,   0, 
      0,   2,   4,   0,   0,   0, 
     15,   0,   0,   8,  18,   0, 
     16,   0,   0,   0,   0,   0, 
      6,  16,  16,   0,   0,   0, 
      0,   0,   6, 128,  32,   0, 
      0,   0,   0,   0,  21,   0, 
      0,   0,   0,   0,   0,   7, 
     18,   0,  16,   0,   0,   0, 
      0,   0,  10,   0,  16,   0, 
      0,   0,   0,   0,   1,  64, 
      0,   0,   0,   0, 128, 191, 
     50,   0,   0,  11,  34,   0, 
     16,   0,   0,   0,   0,   0, 
     26,  16,  16, 128,  65,   0, 
      0,   0,   0,   0,   0,   0, 
     26, 128,  32,   0,   0,   0, 
      0,   0,  21,   0,   0,   0, 
      1,  64,   0,   0,   0,   0, 
    128,  63,  50,   0,   0,   9, 
     34,   0,  16,   0,   0,   0, 
      0,   0,  26,   0,  16,   0, 
      0,   0,   0,   0,   1,  64, 
      0,   0,   0,   0,   0,  64, 
      1,  64,   0,   0,   0,   0, 
    128, 191,  56,   0,   0,   8, 
    226,   0,  16,   0,   0,   0, 
      0,   0,  86,   5,  16,   0, 
      0,   0,   0,   0,   6, 137, 
     32,   0,   0,   0,   0,   0, 
     17,   0,   0,   0,  50,   0, 
      0,  10, 114,   0,  16,   0, 
      0,   0,   0,   0,  70, 130, 
     32,   0,   0,   0,   0,   0, 
     16,   0,   0,   0,   6,   0, 
     16,   0,   0,   0,   0,   0, 
    150,   7,  16,   0,   0,   0, 
      0,   0,   0,   0,   0,   8, 
    114,   0,  16,   0,   0,   0, 
      0,   0,  70,   2,  16,   0, 
      0,   0,   0,   0,  70, 130, 
     32,   0,   0,   0,   0,   0, 
     19,   0,   0,   0,  56,   0, 
      0,   7, 242,   0,  16,   0, 
      1,   0,   0,   0,  86,   5, 
     16,   0,   0,   0,   0,   0, 
     70,  30,  16,   0,   2,   0, 
      0,   0,  50,   0,   0,   9, 
    242,   0,  16,   0,   1,   0, 
      0,   0,  70,  30,  16,   0, 
      1,   0,   0,   0,   6,   0, 
     16,   0,   0,   0,   0,   0, 
     70,  14,  16,   0,   1,   0, 
      0,   0,  50,   0,   0,   9, 
    242,   0,  16,   0,   1,   0, 
      0,   0,  70,  30,  16,   0, 
      3,   0,   0,   0, 166,  10, 
     16,   0,   0,   0,   0,   0, 
     70,  14,  16,   0,   1,   0, 
      0,   0,  16,   0,   0,   7, 
    130,   0,  16,   0,   0,   0, 
      0,   0,  70,   2,  16,   0, 
      1,   0,   0,   0,  70,   2, 
     16,   0,   1,   0,   0,   0, 
     16,   0,   0,   7,  18,   0, 
     16,   0,   1,   0,   0,   0, 
     70,   2,  16,   0,   1,   0, 
      0,   0,  70,  18,  16,   0, 
      4,   0,   0,   0,  50,   0, 
      0,  10,  18,   0,  16,   0, 
      1,   0,   0,   0,  58,   0, 
     16, 128,  65,   0,   0,   0, 
      1,   0,   0,   0,  58,  16, 
     16,   0,   4,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,  16,   0,   0,   7, 
     34,   0,  16,   0,   1,   0, 
      0,   0,  70,  18,  16,   0, 
      4,   0,   0,   0,  70,  18, 
     16,   0,   4,   0,   0,   0, 
     50,   0,   0,  10,  34,   0, 
     16,   0,   1,   0,   0,   0, 
     58,  16,  16, 128,  65,   0, 
      0,   0,   4,   0,   0,   0, 
     58,  16,  16,   0,   4,   0, 
      0,   0,  26,   0,  16,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   7,  66,   0,  16,   0, 
      1,   0,   0,   0,  10,   0, 
     16,   0,   1,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,  24,   0,   0,   7, 
    130,   0,  16,   0,   1,   0, 
      0,   0,  58,   0,  16,   0, 
      0,   0,   0,   0,   1,  64, 
      0,   0,   0,   0,   0,   0, 
     24,   0,   0,   7,  18,   0, 
     16,   0,   2,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0,   0,   0,   1,   0, 
      0,   7, 130,   0,  16,   0, 
      1,   0,   0,   0,  58,   0, 
     16,   0,   1,   0,   0,   0, 
     10,   0,  16,   0,   2,   0, 
      0,   0,  56,   0,   0,   7, 
     18,   0,  16,   0,   2,   0, 
      0,   0,  58,   0,  16,   0, 
      0,   0,   0,   0,  26,   0, 
     16,   0,   1,   0,   0,   0, 
     56,   0,   0,   7,  18,   0, 
     16,   0,   2,   0,   0,   0, 
     10,   0,  16,   0,   2,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128,  64,  50,   0, 
      0,  10,  18,   0,  16,   0, 
      2,   0,   0,   0,  42,   0, 
     16,   0,   1,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,  10,   0,  16, 128, 
     65,   0,   0,   0,   2,   0, 
      0,   0,  49,   0,   0,   7, 
     34,   0,  16,   0,   2,   0, 
      0,   0,  10,   0,  16,   0, 
      2,   0,   0,   0,   1,  64, 
      0,   0,   0,   0,   0,   0, 
     59,   0,   0,   5,  34,   0, 
     16,   0,   3,   0,   0,   0, 
     26,   0,  16,   0,   2,   0, 
      0,   0,  49,   0,   0,   7, 
     18,   0,  16,   0,   1,   0, 
      0,   0,  10,   0,  16,   0, 
      1,   0,   0,   0,   1,  64, 
      0,   0,   0,   0,   0,   0, 
     55,   0,   0,   9,  18,   0, 
     16,   0,   1,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,   1,  64,   0,   0, 
      0,   0, 128, 191,   1,  64, 
      0,   0,   0,   0, 128,  63, 
     75,   0,   0,   5,  18,   0, 
     16,   0,   2,   0,   0,   0, 
     10,   0,  16,   0,   2,   0, 
      0,   0,  50,   0,   0,   9, 
     18,   0,  16,   0,   1,   0, 
      0,   0,  10,   0,  16,   0, 
      1,   0,   0,   0,  10,   0, 
     16,   0,   2,   0,   0,   0, 
     42,   0,  16,   0,   1,   0, 
      0,   0,  56,   0,   0,   7, 
     18,   0,  16,   0,   1,   0, 
      0,   0,  10,   0,  16,   0, 
      1,   0,   0,   0,   1,  64, 
      0,   0,   0,   0,   0, 191, 
     14,   0,   0,   7, 130,   0, 
     16,   0,   0,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,  58,   0,  16,   0, 
      0,   0,   0,   0,  14,   0, 
      0,   7,  18,   0,  16,   0, 
      1,   0,   0,   0,  26,   0, 
     16,   0,   1,   0,   0,   0, 
     10,   0,  16,   0,   1,   0, 
      0,   0,  49,   0,   0,   7, 
     34,   0,  16,   0,   1,   0, 
      0,   0,  10,   0,  16,   0, 
      1,   0,   0,   0,  58,   0, 
     16,   0,   0,   0,   0,   0, 
     55,   0,   0,   9,  66,   0, 
     16,   0,   3,   0,   0,   0, 
     26,   0,  16,   0,   1,   0, 
      0,   0,  10,   0,  16,   0, 
      1,   0,   0,   0,  58,   0, 
     16,   0,   0,   0,   0,   0, 
     54,   0,   0,   8, 146,   0, 
     16,   0,   3,   0,   0,   0, 
      2,  64,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 255, 255, 
    255, 255,  55,   0,   0,   9, 
     50,   0,  16,   0,   1,   0, 
      0,   0,  86,   5,  16,   0, 
      2,   0,   0,   0,  70,   0, 
     16,   0,   3,   0,   0,   0, 
    230,  10,  16,   0,   3,   0, 
      0,   0,  55,   0,   0,  12, 
     50,   0,  16,   0,   1,   0, 
      0,   0, 246,  15,  16,   0, 
      1,   0,   0,   0,   2,  64, 
      0,   0,   0,   0,   0,   0, 
    255, 255, 255, 255,   0,   0, 
      0,   0,   0,   0,   0,   0, 
     70,   0,  16,   0,   1,   0, 
      0,   0,  59,   0,   0,   5, 
    130,   0,  16,   0,   0,   0, 
      0,   0,  26,   0,  16,   0, 
      1,   0,   0,   0,  13,   0, 
      4,   3,  58,   0,  16,   0, 
      0,   0,   0,   0,  56,   0, 
      0,   7, 114,   0,  16,   0, 
      0,   0,   0,   0,  70,   2, 
     16,   0,   0,   0,   0,   0, 
      6,   0,  16,   0,   1,   0, 
      0,   0,  56,   0,   0,   8, 
    162,   0,  16,   0,   0,   0, 
      0,   0,  86,   5,  16,   0, 
      0,   0,   0,   0, 166, 142, 
     32,   0,   0,   0,   0,   0, 
      9,   0,   0,   0,  50,   0, 
      0,  10,  50,   0,  16,   0, 
      0,   0,   0,   0, 230, 138, 
     32,   0,   0,   0,   0,   0, 
      8,   0,   0,   0,   6,   0, 
     16,   0,   0,   0,   0,   0, 
    214,   5,  16,   0,   0,   0, 
      0,   0,  50,   0,   0,  10, 
     50,   0,  16,   0,   0,   0, 
      0,   0, 230, 138,  32,   0, 
      0,   0,   0,   0,  10,   0, 
      0,   0, 166,  10,  16,   0, 
      0,   0,   0,   0,  70,   0, 
     16,   0,   0,   0,   0,   0, 
      0,   0,   0,   8,  50,   0, 
     16,   0,   0,   0,   0,   0, 
     70,   0,  16,   0,   0,   0, 
      0,   0, 230, 138,  32,   0, 
      0,   0,   0,   0,  11,   0, 
      0,   0,  14,   0,   0,   6, 
      1, 192,   0,   0,  10,   0, 
     16,   0,   0,   0,   0,   0, 
     26,   0,  16,   0,   0,   0, 
      0,   0,  54,   0,   0,   5, 
     18,  32,  16,   0,   0,   0, 
      0,   0,  42,   0,  16,   0, 
      0,   0,   0,   0,  62,   0, 
      0,   1,  83,  84,  65,  84, 
    148,   0,   0,   0,  46,   0, 
      0,   0,   4,   0,   0,   0, 
      0,   0,   0,   0,   7,   0, 
      0,   0,  35,   0,   0,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,   1,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   2,   0,   0,   0, 
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
      0,   0
};
