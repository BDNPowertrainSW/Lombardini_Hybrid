function [names, packed_data_types, start_bits, widths, endians, packed_bytes, scales, offsets, signal_data_types] = Lombardini_dual_fuel_pack_example()

packed_bytes = 8;

names = {
'ain00'
'ain01'
'ain02'
'ain03'
};

%    SS_DOUBLE  =  0,    /* real64_T  */
%    SS_SINGLE  =  1,    /* real32_T  */
%    SS_INT8    =  2,    /* int8_T    */
%    SS_UINT8   =  3,    /* uint8_T   */
%    SS_INT16   =  4,    /* int16_T   */
%    SS_UINT16  =  5,    /* uint16_T  */
%    SS_INT32   =  6,    /* int32_T   */
%    SS_UINT32  =  7,    /* uint32_T  */
%    SS_BOOLEAN =  8     /* boolean_T */
packed_data_types  = [
5
5
5
5
];

%    SS_DYNAMICALLY_TYPED  =  -1,    /* real_T    */
signal_data_types  = [
0
0
0
0
];

start_bits  = [
48
32
16
0
];

widths      = [
16
16
16
16
];

% 0 - Big Endian
% 1 - Little Endian
endians     = [
0
0
0
0
];

scales     = [
5/1024
5/1024
5/1024
5/1024
];

offsets     = [
0
0
0
0
];


