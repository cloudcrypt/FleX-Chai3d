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
// POSITION                 0   xyzw        0     NONE   float   xyzw
// DENSITY                  0   x           1     NONE   float   x   
// PHASE                    0   x           2     NONE     int   x   
// SV_VertexID              0   x           3   VERTID    uint       
//
//
// Output signature:
//
// Name                 Index   Mask Register SysValue  Format   Used
// -------------------- ----- ------ -------- -------- ------- ------
// POSITION                 0   xyzw        0     NONE   float   xyzw
// DENSITY                  0   x           1     NONE   float   x   
// PHASE                    0   x           2     NONE     int   x   
// VERTEX                   0   xyzw        3     NONE   float   xyzw
//
vs_5_0
dcl_globalFlags refactoringAllowed
dcl_constantbuffer cb0[4], immediateIndexed
dcl_input v0.xyzw
dcl_input v1.x
dcl_input v2.x
dcl_output o0.xyzw
dcl_output o1.x
dcl_output o2.x
dcl_output o3.xyzw
dcl_temps 1
mul r0.xyzw, v0.yyyy, cb0[1].xyzw
mad r0.xyzw, cb0[0].xyzw, v0.xxxx, r0.xyzw
mad r0.xyzw, cb0[2].xyzw, v0.zzzz, r0.xyzw
add o0.xyzw, r0.xyzw, cb0[3].xyzw
mov o1.x, v1.x
mov o2.x, v2.x
mov o3.xyzw, v0.xyzw
ret 
// Approximately 8 instruction slots used
#endif

const BYTE g_pointVS[] =
{
     68,  88,  66,  67, 214,  36, 
    191, 126, 186,  99, 189,  31, 
     31, 225,  65, 123, 198,  88, 
    217, 159,   1,   0,   0,   0, 
    192,   6,   0,   0,   5,   0, 
      0,   0,  52,   0,   0,   0, 
    176,   3,   0,   0,  68,   4, 
      0,   0, 212,   4,   0,   0, 
     36,   6,   0,   0,  82,  68, 
     69,  70, 116,   3,   0,   0, 
      1,   0,   0,   0, 104,   0, 
      0,   0,   1,   0,   0,   0, 
     60,   0,   0,   0,   0,   5, 
    254, 255,   0,   1,   0,   0, 
     64,   3,   0,   0,  82,  68, 
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
      0,   0, 112,   2,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0, 168,   0,   0,   0, 
      0,   0,   0,   0, 100,   2, 
      0,   0,   2,   0,   0,   0, 
     28,   3,   0,   0,   0,   0, 
      0,   0, 255, 255, 255, 255, 
      0,   0,   0,   0, 255, 255, 
    255, 255,   0,   0,   0,   0, 
    103,  80,  97, 114,  97, 109, 
    115,   0,  80, 111, 105, 110, 
    116,  83, 104,  97, 100, 101, 
    114,  67, 111, 110, 115, 116, 
      0, 109, 111, 100, 101, 108, 
     86, 105, 101, 119,   0, 102, 
    108, 111,  97, 116,  52, 120, 
     52,   0,   3,   0,   3,   0, 
      4,   0,   4,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0, 203,   0, 
      0,   0, 112, 114, 111, 106, 
    101,  99, 116, 105, 111, 110, 
      0, 108, 105, 103, 104, 116, 
     84, 114,  97, 110, 115, 102, 
    111, 114, 109,   0,  99, 111, 
    108, 111, 114, 115,   0, 102, 
    108, 111,  97, 116,  52,   0, 
      1,   0,   3,   0,   1,   0, 
      4,   0,   8,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,  25,   1,   0,   0, 
    115, 104,  97, 100, 111, 119, 
     84,  97, 112, 115,   0, 171, 
      1,   0,   3,   0,   1,   0, 
      4,   0,  12,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,  25,   1,   0,   0, 
    108, 105, 103, 104, 116,  80, 
    111, 115,   0, 102, 108, 111, 
     97, 116,  51,   0,   1,   0, 
      3,   0,   1,   0,   3,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
    125,   1,   0,   0,  95, 112, 
     97, 100,  48,   0, 102, 108, 
    111,  97, 116,   0,   0,   0, 
      3,   0,   1,   0,   1,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
    174,   1,   0,   0, 108, 105, 
    103, 104, 116,  68, 105, 114, 
      0,  95, 112,  97, 100,  49, 
      0, 112, 111, 105, 110, 116, 
     82,  97, 100, 105, 117, 115, 
      0, 112, 111, 105, 110, 116, 
     83,  99,  97, 108, 101,   0, 
    115, 112, 111, 116,  77, 105, 
    110,   0, 115, 112, 111, 116, 
     77,  97, 120,   0, 109, 111, 
    100, 101,   0, 105, 110, 116, 
      0, 171,   0,   0,   2,   0, 
      1,   0,   1,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,  19,   2, 
      0,   0,  95, 112,  97, 100, 
     50,   0, 171, 171,   0,   0, 
      2,   0,   1,   0,   1,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
     19,   2,   0,   0, 193,   0, 
      0,   0, 212,   0,   0,   0, 
      0,   0,   0,   0, 248,   0, 
      0,   0, 212,   0,   0,   0, 
     64,   0,   0,   0,   3,   1, 
      0,   0, 212,   0,   0,   0, 
    128,   0,   0,   0,  18,   1, 
      0,   0,  32,   1,   0,   0, 
    192,   0,   0,   0,  68,   1, 
      0,   0,  80,   1,   0,   0, 
     64,   1,   0,   0, 116,   1, 
      0,   0, 132,   1,   0,   0, 
      0,   2,   0,   0, 168,   1, 
      0,   0, 180,   1,   0,   0, 
     12,   2,   0,   0, 216,   1, 
      0,   0, 132,   1,   0,   0, 
     16,   2,   0,   0, 225,   1, 
      0,   0, 180,   1,   0,   0, 
     28,   2,   0,   0, 231,   1, 
      0,   0, 180,   1,   0,   0, 
     32,   2,   0,   0, 243,   1, 
      0,   0, 180,   1,   0,   0, 
     36,   2,   0,   0, 254,   1, 
      0,   0, 180,   1,   0,   0, 
     40,   2,   0,   0,   6,   2, 
      0,   0, 180,   1,   0,   0, 
     44,   2,   0,   0,  14,   2, 
      0,   0,  24,   2,   0,   0, 
     48,   2,   0,   0,  60,   2, 
      0,   0,  68,   2,   0,   0, 
     64,   2,   0,   0,   5,   0, 
      0,   0,   1,   0, 144,   0, 
      0,   0,  15,   0, 104,   2, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
    176,   0,   0,   0,  77, 105, 
     99, 114, 111, 115, 111, 102, 
    116,  32,  40,  82,  41,  32, 
     72,  76,  83,  76,  32,  83, 
    104,  97, 100, 101, 114,  32, 
     67, 111, 109, 112, 105, 108, 
    101, 114,  32,  54,  46,  51, 
     46,  57,  54,  48,  48,  46, 
     49,  54,  51,  56,  52,   0, 
    171, 171,  73,  83,  71,  78, 
    140,   0,   0,   0,   4,   0, 
      0,   0,   8,   0,   0,   0, 
    104,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      3,   0,   0,   0,   0,   0, 
      0,   0,  15,  15,   0,   0, 
    113,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      3,   0,   0,   0,   1,   0, 
      0,   0,   1,   1,   0,   0, 
    121,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      2,   0,   0,   0,   2,   0, 
      0,   0,   1,   1,   0,   0, 
    127,   0,   0,   0,   0,   0, 
      0,   0,   6,   0,   0,   0, 
      1,   0,   0,   0,   3,   0, 
      0,   0,   1,   0,   0,   0, 
     80,  79,  83,  73,  84,  73, 
     79,  78,   0,  68,  69,  78, 
     83,  73,  84,  89,   0,  80, 
     72,  65,  83,  69,   0,  83, 
     86,  95,  86, 101, 114, 116, 
    101, 120,  73,  68,   0, 171, 
     79,  83,  71,  78, 136,   0, 
      0,   0,   4,   0,   0,   0, 
      8,   0,   0,   0, 104,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,   0,   0,   0,   0, 
     15,   0,   0,   0, 113,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,   1,   0,   0,   0, 
      1,  14,   0,   0, 121,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   2,   0, 
      0,   0,   2,   0,   0,   0, 
      1,  14,   0,   0, 127,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,   3,   0,   0,   0, 
     15,   0,   0,   0,  80,  79, 
     83,  73,  84,  73,  79,  78, 
      0,  68,  69,  78,  83,  73, 
     84,  89,   0,  80,  72,  65, 
     83,  69,   0,  86,  69,  82, 
     84,  69,  88,   0, 171, 171, 
     83,  72,  69,  88,  72,   1, 
      0,   0,  80,   0,   1,   0, 
     82,   0,   0,   0, 106,   8, 
      0,   1,  89,   0,   0,   4, 
     70, 142,  32,   0,   0,   0, 
      0,   0,   4,   0,   0,   0, 
     95,   0,   0,   3, 242,  16, 
     16,   0,   0,   0,   0,   0, 
     95,   0,   0,   3,  18,  16, 
     16,   0,   1,   0,   0,   0, 
     95,   0,   0,   3,  18,  16, 
     16,   0,   2,   0,   0,   0, 
    101,   0,   0,   3, 242,  32, 
     16,   0,   0,   0,   0,   0, 
    101,   0,   0,   3,  18,  32, 
     16,   0,   1,   0,   0,   0, 
    101,   0,   0,   3,  18,  32, 
     16,   0,   2,   0,   0,   0, 
    101,   0,   0,   3, 242,  32, 
     16,   0,   3,   0,   0,   0, 
    104,   0,   0,   2,   1,   0, 
      0,   0,  56,   0,   0,   8, 
    242,   0,  16,   0,   0,   0, 
      0,   0,  86,  21,  16,   0, 
      0,   0,   0,   0,  70, 142, 
     32,   0,   0,   0,   0,   0, 
      1,   0,   0,   0,  50,   0, 
      0,  10, 242,   0,  16,   0, 
      0,   0,   0,   0,  70, 142, 
     32,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   6,  16, 
     16,   0,   0,   0,   0,   0, 
     70,  14,  16,   0,   0,   0, 
      0,   0,  50,   0,   0,  10, 
    242,   0,  16,   0,   0,   0, 
      0,   0,  70, 142,  32,   0, 
      0,   0,   0,   0,   2,   0, 
      0,   0, 166,  26,  16,   0, 
      0,   0,   0,   0,  70,  14, 
     16,   0,   0,   0,   0,   0, 
      0,   0,   0,   8, 242,  32, 
     16,   0,   0,   0,   0,   0, 
     70,  14,  16,   0,   0,   0, 
      0,   0,  70, 142,  32,   0, 
      0,   0,   0,   0,   3,   0, 
      0,   0,  54,   0,   0,   5, 
     18,  32,  16,   0,   1,   0, 
      0,   0,  10,  16,  16,   0, 
      1,   0,   0,   0,  54,   0, 
      0,   5,  18,  32,  16,   0, 
      2,   0,   0,   0,  10,  16, 
     16,   0,   2,   0,   0,   0, 
     54,   0,   0,   5, 242,  32, 
     16,   0,   3,   0,   0,   0, 
     70,  30,  16,   0,   0,   0, 
      0,   0,  62,   0,   0,   1, 
     83,  84,  65,  84, 148,   0, 
      0,   0,   8,   0,   0,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   7,   0,   0,   0, 
      4,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      1,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      0,   0,   0,   0,   0,   0, 
      3,   0,   0,   0,   0,   0, 
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
