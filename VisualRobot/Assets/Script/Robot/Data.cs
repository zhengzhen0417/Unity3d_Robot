﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace robot
{
   public class Data
    {
        public static double[,] DataBuffer = new double [,]{
           {0,0,30,0},
{30.24,197.91,30,0},
{30.24,197.91,0,0},
{26.46,198.42,0,1},
{24.66,198.81,0,1},
{22.95,199.29,0,1},
{21.27,199.83,0,1},
{19.68,200.49,0,1},
{18.15,201.21,0,1},
{16.68,202.05,0,1},
{15.27,202.95,0,1},
{13.92,203.91,0,1},
{12.6,204.99,0,1},
{11.37,206.13,0,1},
{10.17,207.39,0,1},
{9.059999,208.71,0,1},
{7.86,210.3,0,1},
{6.75,211.98,0,1},
{5.73,213.72,0,1},
{4.8,215.55,0,1},
{3.99,217.47,0,1},
{3.24,219.45,0,1},
{2.58,221.52,0,1},
{2.04,223.65,0,1},
{1.14,228.24,0,1},
{0.51,233.34,0,1},
{0.12,238.95,0,1},
{0,245.07,0,1},
{0,319.53,0,1},
{12.6,319.53,0,1},
{12.6,245.07,0,1},
{12.81,237.48,0,1},
{13.44,231.18,0,1},
{13.95,228.42,0,1},
{14.67,225.81,0,1},
{15.57,223.35,0,1},
{16.68,221.04,0,1},
{17.34,219.87,0,1},
{18.06,218.79,0,1},
{18.84,217.77,0,1},
{19.68,216.81,0,1},
{20.58,215.94,0,1},
{21.54,215.13,0,1},
{22.53,214.41,0,1},
{23.61,213.75,0,1},
{24.72,213.15,0,1},
{25.92,212.67,0,1},
{27.15,212.22,0,1},
{28.44,211.89,0,1},
{29.82,211.62,0,1},
{31.23,211.41,0,1},
{32.73,211.29,0,1},
{34.29,211.26,0,1},
{35.73,211.29,0,1},
{37.17,211.41,0,1},
{38.55,211.62,0,1},
{39.87,211.89,0,1},
{41.16,212.22,0,1},
{42.42,212.67,0,1},
{43.62,213.15,0,1},
{44.79,213.75,0,1},
{45.9,214.41,0,1},
{46.95,215.13,0,1},
{47.94,215.94,0,1},
{48.87,216.81,0,1},
{49.74,217.77,0,1},
{50.52,218.79,0,1},
{51.27,219.87,0,1},
{51.93,221.04,0,1},
{53.01,223.38,0,1},
{53.91,225.84,0,1},
{54.63,228.42,0,1},
{55.14,231.12,0,1},
{55.8,237.27,0,1},
{56.01,244.68,0,1},
{56.01,319.53,0,1},
{68.61,319.53,0,1},
{68.61,245.07,0,1},
{68.49,239.1,0,1},
{68.13,233.55,0,1},
{67.5,228.42,0,1},
{66.6,223.77,0,1},
{66.06,221.58,0,1},
{65.43,219.48,0,1},
{64.68,217.47,0,1},
{63.84,215.55,0,1},
{62.91,213.72,0,1},
{61.89,211.95,0,1},
{60.78,210.3,0,1},
{59.58,208.71,0,1},
{58.41,207.36,0,1},
{57.21,206.1,0,1},
{55.98,204.96,0,1},
{54.66,203.88,0,1},
{53.34,202.89,0,1},
{51.93,201.99,0,1},
{50.52,201.18,0,1},
{49.02,200.46,0,1},
{47.49,199.8,0,1},
{45.84,199.26,0,1},
{44.13,198.78,0,1},
{42.33,198.39,0,1},
{38.46,197.88,0,1},
{34.29,197.73,0,1},
{34.29,197.73,30,0},
{129.51,200.22,30,0},
{129.51,200.22,0,0},
{129.51,251.19,0,1},
{129.39,257.16,0,1},
{129.03,262.68,0,1},
{128.76,265.23,0,1},
{128.34,267.48,0,1},
{127.83,269.49,0,1},
{127.23,271.23,0,1},
{126.84,272.04,0,1},
{126.42,272.79,0,1},
{125.97,273.51,0,1},
{125.46,274.17,0,1},
{124.95,274.77,0,1},
{124.38,275.31,0,1},
{123.78,275.79,0,1},
{123.15,276.24,0,1},
{122.46,276.63,0,1},
{121.71,276.96,0,1},
{120.9,277.23,0,1},
{120.06,277.47,0,1},
{119.13,277.65,0,1},
{118.14,277.77,0,1},
{117.12,277.86,0,1},
{116.01,277.89,0,1},
{114.87,277.83,0,1},
{113.76,277.68,0,1},
{112.62,277.47,0,1},
{111.48,277.14,0,1},
{110.31,276.72,0,1},
{109.14,276.18,0,1},
{107.97,275.58,0,1},
{106.8,274.86,0,1},
{104.43,273.27,0,1},
{102.09,271.41,0,1},
{99.78,269.34,0,1},
{97.5,267.06,0,1},
{97.5,200.22,0,1},
{85.53,200.22,0,1},
{85.53,289.74,0,1},
{97.5,289.74,0,1},
{97.5,279.81,0,1},
{100.17,282.63,0,1},
{102.84,285.12,0,1},
{105.48,287.25,0,1},
{108.09,289.02,0,1},
{109.41,289.77,0,1},
{110.73,290.43,0,1},
{112.08,290.97,0,1},
{113.46,291.42,0,1},
{114.84,291.78,0,1},
{116.25,292.02,0,1},
{117.69,292.17,0,1},
{119.13,292.23,0,1},
{120.42,292.2,0,1},
{121.68,292.08,0,1},
{122.91,291.9,0,1},
{124.08,291.69,0,1},
{125.25,291.36,0,1},
{126.36,291,0,1},
{127.44,290.55,0,1},
{128.49,290.04,0,1},
{129.51,289.47,0,1},
{130.47,288.81,0,1},
{131.4,288.12,0,1},
{132.3,287.34,0,1},
{133.17,286.47,0,1},
{134.01,285.57,0,1},
{134.79,284.58,0,1},
{135.57,283.53,0,1},
{136.29,282.42,0,1},
{136.95,281.22,0,1},
{137.58,279.99,0,1},
{138.15,278.7,0,1},
{139.17,275.91,0,1},
{140.01,272.88,0,1},
{140.64,269.61,0,1},
{141.12,266.1,0,1},
{141.39,262.32,0,1},
{141.48,258.33,0,1},
{141.48,200.22,0,1},
{141.48,200.22,30,0},
{158.37,200.22,30,0},
{158.37,200.22,0,0},
{158.37,289.74,0,1},
{170.34,289.74,0,1},
{170.34,200.22,0,1},
{170.34,200.22,30,0},
{157.56,304.68,30,0},
{157.56,304.68,0,0},
{157.56,320.31,0,1},
{171.21,320.31,0,1},
{171.21,304.68,0,1},
{171.21,304.68,30,0},
{207.03,198.54,30,0},
{207.03,198.54,0,0},
{204.78,198.87,0,1},
{203.7,199.11,0,1},
{202.65,199.41,0,1},
{201.66,199.77,0,1},
{200.67,200.19,0,1},
{199.74,200.64,0,1},
{198.84,201.18,0,1},
{197.94,201.75,0,1},
{197.13,202.38,0,1},
{196.32,203.07,0,1},
{195.54,203.79,0,1},
{194.79,204.6,0,1},
{194.1,205.44,0,1},
{193.44,206.37,0,1},
{192.81,207.33,0,1},
{192.24,208.35,0,1},
{191.7,209.46,0,1},
{190.77,211.86,0,1},
{189.99,214.5,0,1},
{189.39,217.38,0,1},
{188.97,220.53,0,1},
{188.7,223.95,0,1},
{188.61,227.61,0,1},
{188.61,277.2,0,1},
{180.51,277.2,0,1},
{180.51,289.74,0,1},
{188.61,289.74,0,1},
{188.61,315.48,0,1},
{200.58,315.48,0,1},
{200.58,289.74,0,1},
{222.69,289.74,0,1},
{222.69,277.2,0,1},
{200.58,277.2,0,1},
{200.58,234.66,0,1},
{200.64,228.63,0,1},
{200.79,224.16,0,1},
{200.94,222.36,0,1},
{201.24,220.59,0,1},
{201.66,218.91,0,1},
{202.23,217.26,0,1},
{202.53,216.6,0,1},
{202.86,215.97,0,1},
{203.25,215.37,0,1},
{203.67,214.83,0,1},
{204.12,214.32,0,1},
{204.63,213.84,0,1},
{205.17,213.45,0,1},
{205.77,213.06,0,1},
{206.4,212.73,0,1},
{207.09,212.46,0,1},
{207.81,212.22,0,1},
{208.62,212.01,0,1},
{209.46,211.86,0,1},
{210.36,211.74,0,1},
{211.32,211.68,0,1},
{212.34,211.65,0,1},
{213.78,211.74,0,1},
{215.22,211.92,0,1},
{216.63,212.28,0,1},
{218.01,212.76,0,1},
{220.38,213.75,0,1},
{221.97,214.56,0,1},
{222.69,214.56,0,1},
{222.69,201,0,1},
{219.27,199.92,0,1},
{215.82,199.11,0,1},
{212.49,198.6,0,1},
{209.4,198.42,0,1},
{209.4,198.42,30,0},
{249.06,167.22,30,0},
{249.06,167.22,0,0},
{236.25,167.22,0,1},
{249,204.54,0,1},
{223.14,289.74,0,1},
{236.07,289.74,0,1},
{255.51,223.41,0,1},
{274.86,289.74,0,1},
{287.34,289.74,0,1},
{287.34,289.74,30,0},
{318.42,308.1,30,0},
{318.42,308.1,0,0},
{316.68,307.89,0,1},
{314.97,307.59,0,1},
{313.26,307.11,0,1},
{309.84,305.91,0,1},
{306.48,304.41,0,1},
{303.63,302.76,0,1},
{301.17,301.02,0,1},
{299.04,299.31,0,1},
{297.12,297.75,0,1},
{296.37,297.75,0,1},
{296.37,314.64,0,1},
{298.62,316.02,0,1},
{301.23,317.31,0,1},
{304.23,318.54,0,1},
{307.59,319.74,0,1},
{311.1,320.73,0,1},
{314.55,321.45,0,1},
{317.88,321.87,0,1},
{321.15,322.02,0,1},
{324.24,321.9,0,1},
{327.15,321.6,0,1},
{329.88,321.09,0,1},
{332.37,320.37,0,1},
{333.57,319.92,0,1},
{334.74,319.44,0,1},
{335.85,318.87,0,1},
{336.96,318.27,0,1},
{339.06,316.89,0,1},
{341.07,315.27,0,1},
{342.09,314.28,0,1},
{343.05,313.26,0,1},
{343.92,312.18,0,1},
{344.76,311.04,0,1},
{345.51,309.84,0,1},
{346.2,308.61,0,1},
{346.83,307.32,0,1},
{347.4,305.97,0,1},
{347.91,304.59,0,1},
{348.33,303.15,0,1},
{348.72,301.65,0,1},
{349.02,300.06,0,1},
{349.41,296.76,0,1},
{349.53,293.25,0,1},
{349.47,290.82,0,1},
{349.23,288.45,0,1},
{348.81,286.17,0,1},
{348.24,283.98,0,1},
{347.52,281.85,0,1},
{346.65,279.81,0,1},
{345.6,277.83,0,1},
{344.37,275.94,0,1},
{343.05,274.14,0,1},
{341.67,272.55,0,1},
{340.23,271.11,0,1},
{338.7,269.85,0,1},
{337.14,268.74,0,1},
{335.49,267.81,0,1},
{333.78,267.06,0,1},
{332.01,266.46,0,1},
{332.01,265.35,0,1},
{333.54,264.93,0,1},
{335.13,264.36,0,1},
{336.75,263.67,0,1},
{338.43,262.86,0,1},
{339.27,262.41,0,1},
{340.11,261.87,0,1},
{340.92,261.3,0,1},
{341.7,260.67,0,1},
{343.26,259.29,0,1},
{344.76,257.67,0,1},
{345.48,256.77,0,1},
{346.17,255.81,0,1},
{346.8,254.79,0,1},
{347.43,253.71,0,1},
{348.54,251.4,0,1},
{349.53,248.79,0,1},
{350.34,245.94,0,1},
{350.91,242.79,0,1},
{351.27,239.37,0,1},
{351.39,235.62,0,1},
{351.24,231.66,0,1},
{350.85,227.85,0,1},
{350.16,224.22,0,1},
{349.2,220.77,0,1},
{347.97,217.47,0,1},
{346.53,214.38,0,1},
{344.88,211.47,0,1},
{342.99,208.77,0,1},
{341.91,207.45,0,1},
{340.8,206.22,0,1},
{339.63,205.08,0,1},
{338.4,204,0,1},
{337.14,203.01,0,1},
{335.79,202.08,0,1},
{334.41,201.27,0,1},
{332.97,200.52,0,1},
{331.5,199.86,0,1},
{329.94,199.29,0,1},
{328.35,198.81,0,1},
{326.73,198.42,0,1},
{325.02,198.12,0,1},
{323.28,197.91,0,1},
{321.51,197.76,0,1},
{319.68,197.73,0,1},
{316.11,197.85,0,1},
{312.57,198.27,0,1},
{309.03,198.93,0,1},
{305.52,199.89,0,1},
{302.16,201.03,0,1},
{299.07,202.26,0,1},
{296.28,203.61,0,1},
{293.73,205.08,0,1},
{293.73,222,0,1},
{294.63,222,0,1},
{296.82,220.17,0,1},
{299.34,218.4,0,1},
{302.19,216.63,0,1},
{305.4,214.95,0,1},
{307.08,214.14,0,1},
{308.79,213.48,0,1},
{310.47,212.88,0,1},
{312.15,212.4,0,1},
{313.83,212.04,0,1},
{315.54,211.77,0,1},
{317.22,211.62,0,1},
{318.9,211.56,0,1},
{320.85,211.68,0,1},
{322.8,211.95,0,1},
{324.75,212.46,0,1},
{326.7,213.12,0,1},
{327.66,213.57,0,1},
{328.59,214.05,0,1},
{329.46,214.59,0,1},
{330.3,215.19,0,1},
{331.08,215.85,0,1},
{331.86,216.57,0,1},
{332.55,217.38,0,1},
{333.24,218.22,0,1},
{334.44,220.02,0,1},
{335.49,221.94,0,1},
{336.39,223.92,0,1},
{337.11,225.99,0,1},
{337.71,228.27,0,1},
{338.13,230.79,0,1},
{338.37,233.61,0,1},
{338.46,236.73,0,1},
{338.37,239.82,0,1},
{338.1,242.64,0,1},
{337.89,243.93,0,1},
{337.62,245.16,0,1},
{337.32,246.3,0,1},
{336.96,247.38,0,1},
{336.57,248.4,0,1},
{336.15,249.36,0,1},
{335.67,250.26,0,1},
{335.16,251.1,0,1},
{334.62,251.91,0,1},
{334.05,252.63,0,1},
{333.45,253.32,0,1},
{332.79,253.95,0,1},
{332.1,254.52,0,1},
{331.38,255.06,0,1},
{330.66,255.54,0,1},
{329.88,255.99,0,1},
{329.07,256.38,0,1},
{328.23,256.71,0,1},
{327.36,257.01,0,1},
{326.46,257.28,0,1},
{324.57,257.67,0,1},
{322.65,257.97,0,1},
{320.61,258.15,0,1},
{318.51,258.21,0,1},
{313.11,258.21,0,1},
{313.11,271.5,0,1},
{317.31,271.5,0,1},
{319.41,271.59,0,1},
{321.42,271.83,0,1},
{323.31,272.22,0,1},
{325.11,272.79,0,1},
{325.98,273.15,0,1},
{326.82,273.51,0,1},
{327.63,273.93,0,1},
{328.41,274.41,0,1},
{329.16,274.89,0,1},
{329.91,275.43,0,1},
{330.63,276.03,0,1},
{331.32,276.63,0,1},
{331.95,277.29,0,1},
{332.58,278.01,0,1},
{333.15,278.73,0,1},
{333.66,279.51,0,1},
{334.14,280.32,0,1},
{334.59,281.16,0,1},
{334.98,282.03,0,1},
{335.34,282.93,0,1},
{335.94,284.88,0,1},
{336.36,286.95,0,1},
{336.6,289.2,0,1},
{336.69,291.57,0,1},
{336.6,293.73,0,1},
{336.33,295.74,0,1},
{336.15,296.7,0,1},
{335.91,297.6,0,1},
{335.61,298.47,0,1},
{335.28,299.31,0,1},
{334.53,300.84,0,1},
{333.69,302.22,0,1},
{333.24,302.85,0,1},
{332.76,303.42,0,1},
{332.25,303.96,0,1},
{331.71,304.47,0,1},
{331.11,304.98,0,1},
{330.48,305.43,0,1},
{329.82,305.85,0,1},
{329.16,306.24,0,1},
{328.5,306.57,0,1},
{327.81,306.87,0,1},
{327.09,307.14,0,1},
{326.37,307.35,0,1},
{324.87,307.71,0,1},
{323.34,307.95,0,1},
{321.75,308.1,0,1},
{320.13,308.16,0,1},
{320.13,308.16,30,0},
{442.8,251.76,30,0},
{442.8,251.76,0,0},
{441.81,244.2,0,1},
{441.06,240.57,0,1},
{440.16,237.03,0,1},
{439.11,233.61,0,1},
{437.88,230.25,0,1},
{436.5,227.07,0,1},
{435.03,224.04,0,1},
{433.41,221.22,0,1},
{431.73,218.55,0,1},
{429.9,216.09,0,1},
{427.95,213.81,0,1},
{425.91,211.68,0,1},
{423.75,209.76,0,1},
{420.15,207.15,0,1},
{416.52,204.99,0,1},
{414.69,204.12,0,1},
{412.83,203.34,0,1},
{411,202.68,0,1},
{409.14,202.14,0,1},
{405.18,201.3,0,1},
{400.71,200.7,0,1},
{395.7,200.34,0,1},
{390.18,200.22,0,1},
{368.34,200.22,0,1},
{368.34,319.53,0,1},
{389.94,319.53,0,1},
{396.27,319.35,0,1},
{401.88,318.9,0,1},
{406.77,318.09,0,1},
{410.94,316.98,0,1},
{414.6,315.6,0,1},
{417.96,314.01,0,1},
{421.08,312.15,0,1},
{423.9,310.05,0,1},
{426.12,308.07,0,1},
{428.22,305.94,0,1},
{430.2,303.66,0,1},
{432.03,301.2,0,1},
{433.74,298.59,0,1},
{435.33,295.83,0,1},
{436.77,292.92,0,1},
{438.09,289.83,0,1},
{439.26,286.59,0,1},
{440.31,283.23,0,1},
{441.15,279.69,0,1},
{441.87,276,0,1},
{442.8,268.17,0,1},
{443.13,259.74,0,1},
{443.13,259.74,30,0},
{429.75,266.61,30,0},
{429.75,266.61,0,0},
{429.09,272.76,0,1},
{428.58,275.64,0,1},
{427.95,278.37,0,1},
{427.2,281.01,0,1},
{426.36,283.5,0,1},
{425.4,285.87,0,1},
{424.32,288.09,0,1},
{423.15,290.19,0,1},
{421.86,292.17,0,1},
{420.45,294,0,1},
{418.95,295.68,0,1},
{417.33,297.24,0,1},
{415.59,298.68,0,1},
{413.04,300.48,0,1},
{410.4,301.98,0,1},
{407.67,303.24,0,1},
{404.82,304.2,0,1},
{401.79,304.95,0,1},
{398.4,305.46,0,1},
{394.65,305.79,0,1},
{390.57,305.88,0,1},
{380.94,305.88,0,1},
{380.94,213.84,0,1},
{390.57,213.84,0,1},
{394.65,213.96,0,1},
{398.46,214.26,0,1},
{402,214.8,0,1},
{405.27,215.52,0,1},
{406.83,216,0,1},
{408.36,216.54,0,1},
{409.86,217.2,0,1},
{411.3,217.92,0,1},
{412.74,218.76,0,1},
{414.12,219.66,0,1},
{415.47,220.68,0,1},
{416.82,221.76,0,1},
{418.41,223.23,0,1},
{419.88,224.82,0,1},
{421.26,226.5,0,1},
{422.55,228.3,0,1},
{423.75,230.25,0,1},
{424.83,232.29,0,1},
{425.79,234.45,0,1},
{426.69,236.73,0,1},
{428.13,241.68,0,1},
{429.15,247.2,0,1},
{429.78,253.32,0,1},
{429.99,259.98,0,1},
{429.99,259.98,30,0},
{58.23,2.49,30,0},
{58.23,2.49,0,0},
{28.23,49.92,0,1},
{13.77,49.92,0,1},
{13.77,2.49,0,1},
{1.17,2.49,0,1},
{1.17,121.8,0,1},
{25.62,121.8,0,1},
{33.12,121.5,0,1},
{39.27,120.66,0,1},
{40.65,120.36,0,1},
{42,119.94,0,1},
{43.32,119.43,0,1},
{44.64,118.8,0,1},
{45.96,118.11,0,1},
{47.22,117.33,0,1},
{48.51,116.43,0,1},
{49.74,115.47,0,1},
{50.97,114.42,0,1},
{52.11,113.31,0,1},
{53.19,112.14,0,1},
{54.21,110.88,0,1},
{55.14,109.56,0,1},
{56.04,108.15,0,1},
{56.85,106.68,0,1},
{57.6,105.12,0,1},
{58.29,103.5,0,1},
{58.89,101.76,0,1},
{59.37,99.96,0,1},
{59.79,98.04,0,1},
{60.33,93.93,0,1},
{60.51,89.43,0,1},
{60.42,86.22,0,1},
{60.18,83.13,0,1},
{59.76,80.18999,0,1},
{59.19,77.37,0,1},
{58.44,74.7,0,1},
{57.54,72.15,0,1},
{56.46,69.75,0,1},
{55.23,67.47,0,1},
{53.85,65.34,0,1},
{52.35,63.36,0,1},
{50.73,61.5,0,1},
{48.99,59.79,0,1},
{47.13,58.23,0,1},
{45.15,56.82,0,1},
{43.05,55.53,0,1},
{40.86,54.42,0,1},
{74.61,2.49,0,1},
{74.61,2.49,30,0},
{47.25,90.75,30,0},
{47.25,90.75,0,0},
{46.98,93.15,0,1},
{46.53,95.37,0,1},
{45.87,97.47,0,1},
{45.48,98.46,0,1},
{45.03,99.39,0,1},
{44.55,100.29,0,1},
{44.01,101.16,0,1},
{43.41,101.94,0,1},
{42.78,102.72,0,1},
{42.09,103.41,0,1},
{41.34,104.07,0,1},
{40.68,104.61,0,1},
{39.96,105.12,0,1},
{39.21,105.57,0,1},
{38.46,105.99,0,1},
{37.65,106.38,0,1},
{36.84,106.71,0,1},
{36,107.01,0,1},
{35.1,107.25,0,1},
{33.21,107.64,0,1},
{31.11,107.94,0,1},
{28.8,108.12,0,1},
{26.25,108.18,0,1},
{13.77,108.18,0,1},
{13.77,63.15,0,1},
{24.87,63.15,0,1},
{27.48,63.21,0,1},
{30,63.45,0,1},
{32.37,63.81,0,1},
{34.59,64.35,0,1},
{35.67,64.68,0,1},
{36.69,65.1,0,1},
{37.68,65.61,0,1},
{38.61,66.18,0,1},
{39.51,66.84,0,1},
{40.38,67.59,0,1},
{41.19,68.4,0,1},
{42,69.3,0,1},
{42.66,70.17,0,1},
{43.26,71.07,0,1},
{43.83,72.03,0,1},
{44.37,72.99,0,1},
{45.3,75.09,0,1},
{46.02,77.34,0,1},
{46.59,79.74,0,1},
{47.01,82.38,0,1},
{47.25,85.2,0,1},
{47.34,88.23,0,1},
{47.34,88.23,30,0},
{139.32,41.73,30,0},
{139.32,41.73,0,0},
{138.93,36.54,0,1},
{138.24,31.71,0,1},
{137.31,27.18,0,1},
{136.08,22.98,0,1},
{134.61,19.11,0,1},
{133.74,17.28,0,1},
{132.84,15.54,0,1},
{131.85,13.89,0,1},
{130.8,12.3,0,1},
{129.69,10.8,0,1},
{128.55,9.42,0,1},
{127.35,8.13,0,1},
{126.12,6.93,0,1},
{124.83,5.82,0,1},
{123.51,4.8,0,1},
{122.13,3.9,0,1},
{120.72,3.09,0,1},
{119.25,2.34,0,1},
{117.75,1.74,0,1},
{116.19,1.2,0,1},
{114.6,0.78,0,1},
{112.95,0.42,0,1},
{111.27,0.18,0,1},
{109.53,0.06,0,1},
{107.76,0,0,1},
{105.93,0.06,0,1},
{104.13,0.21,0,1},
{102.42,0.45,0,1},
{100.74,0.78,0,1},
{99.09,1.23,0,1},
{97.53,1.8,0,1},
{95.97,2.43,0,1},
{94.5,3.18,0,1},
{93.06,4.02,0,1},
{91.68,4.95,0,1},
{90.36,6,0,1},
{89.07,7.14,0,1},
{87.84,8.4,0,1},
{86.64,9.719999,0,1},
{85.5,11.16,0,1},
{84.42,12.69,0,1},
{82.41,16.02,0,1},
{80.7,19.62,0,1},
{79.23,23.52,0,1},
{78.03,27.69,0,1},
{77.1,32.13,0,1},
{76.43999,36.87,0,1},
{76.05,41.91,0,1},
{75.9,47.22,0,1},
{76.05,52.65,0,1},
{76.47,57.81,0,1},
{77.13,62.64,0,1},
{78.09,67.14,0,1},
{79.32,71.34,0,1},
{80.85,75.24,0,1},
{82.62,78.81,0,1},
{84.66,82.08,0,1},
{85.8,83.58,0,1},
{86.93999,84.99,0,1},
{88.14,86.31,0,1},
{89.4,87.50999,0,1},
{90.68999,88.62,0,1},
{92.00999,89.64,0,1},
{93.39,90.57,0,1},
{94.82999,91.38,0,1},
{96.27,92.13,0,1},
{97.8,92.75999,0,1},
{99.32999,93.27,0,1},
{100.95,93.72,0,1},
{102.57,94.05,0,1},
{104.25,94.29,0,1},
{105.99,94.43999,0,1},
{107.76,94.5,0,1},
{109.53,94.43999,0,1},
{111.27,94.29,0,1},
{112.95,94.05,0,1},
{114.6,93.72,0,1},
{116.19,93.3,0,1},
{117.75,92.75999,0,1},
{119.25,92.13,0,1},
{120.72,91.41,0,1},
{122.13,90.57,0,1},
{123.51,89.67,0,1},
{124.83,88.65,0,1},
{126.12,87.54,0,1},
{127.35,86.31,0,1},
{128.55,85.02,0,1},
{129.69,83.61,0,1},
{130.8,82.11,0,1},
{132.84,78.87,0,1},
{134.61,75.3,0,1},
{136.08,71.4,0,1},
{137.31,67.2,0,1},
{138.24,62.67,0,1},
{138.93,57.84,0,1},
{139.32,52.68,0,1},
{139.47,47.22,0,1},
{139.47,47.22,30,0},
{126.81,55.35,30,0},
{126.81,55.35,0,0},
{125.85,62.37,0,1},
{125.16,65.46,0,1},
{124.29,68.28,0,1},
{123.27,70.83,0,1},
{122.07,73.08,0,1},
{121.44,74.1,0,1},
{120.75,75.06,0,1},
{120.06,75.96,0,1},
{119.31,76.77,0,1},
{118.53,77.55,0,1},
{117.72,78.24,0,1},
{116.85,78.87,0,1},
{115.98,79.41,0,1},
{115.08,79.92,0,1},
{114.12,80.34,0,1},
{113.16,80.7,0,1},
{112.14,81,0,1},
{111.09,81.24,0,1},
{110.01,81.42,0,1},
{108.9,81.50999,0,1},
{107.76,81.54,0,1},
{106.59,81.50999,0,1},
{105.48,81.42,0,1},
{104.4,81.24,0,1},
{103.35,81,0,1},
{102.33,80.7,0,1},
{101.34,80.34,0,1},
{100.38,79.92,0,1},
{99.48,79.41,0,1},
{98.57999,78.87,0,1},
{97.74,78.24,0,1},
{96.93,77.55,0,1},
{96.12,76.77,0,1},
{95.37,75.96,0,1},
{94.68,75.06,0,1},
{93.99,74.1,0,1},
{93.32999,73.08,0,1},
{92.16,70.83,0,1},
{91.11,68.28,0,1},
{90.24,65.46,0,1},
{89.55,62.37,0,1},
{88.59,55.35,0,1},
{88.25999,47.22,0,1},
{88.59,39.3,0,1},
{89.55,32.4,0,1},
{90.24,29.34,0,1},
{91.11,26.52,0,1},
{92.16,23.97,0,1},
{93.32999,21.66,0,1},
{93.99,20.61,0,1},
{94.68,19.62,0,1},
{95.37,18.69,0,1},
{96.12,17.85,0,1},
{96.93,17.07,0,1},
{97.74,16.35,0,1},
{98.57999,15.72,0,1},
{99.48,15.12,0,1},
{100.38,14.61,0,1},
{101.34,14.19,0,1},
{102.33,13.8,0,1},
{103.35,13.5,0,1},
{104.4,13.26,0,1},
{105.48,13.08,0,1},
{106.59,12.99,0,1},
{107.76,12.96,0,1},
{108.9,12.99,0,1},
{110.01,13.08,0,1},
{111.06,13.26,0,1},
{112.11,13.5,0,1},
{113.13,13.8,0,1},
{114.09,14.16,0,1},
{115.05,14.58,0,1},
{115.95,15.09,0,1},
{116.82,15.66,0,1},
{117.66,16.29,0,1},
{118.47,17.01,0,1},
{119.25,17.76,0,1},
{120,18.6,0,1},
{120.72,19.5,0,1},
{121.41,20.46,0,1},
{122.04,21.51,0,1},
{123.24,23.76,0,1},
{124.26,26.31,0,1},
{125.13,29.13,0,1},
{125.85,32.19,0,1},
{126.81,39.15,0,1},
{127.11,47.22,0,1},
{127.11,47.22,30,0},
{198.84,55.05,30,0},
{198.84,55.05,0,0},
{198.12,61.62,0,1},
{197.58,64.56,0,1},
{196.92,67.23,0,1},
{196.11,69.66,0,1},
{195.21,71.85,0,1},
{194.7,72.87,0,1},
{194.16,73.8,0,1},
{193.59,74.67,0,1},
{192.96,75.48,0,1},
{192.33,76.23,0,1},
{191.64,76.92,0,1},
{190.89,77.52,0,1},
{190.14,78.09,0,1},
{189.33,78.57,0,1},
{188.52,78.99,0,1},
{187.62,79.35,0,1},
{186.72,79.65,0,1},
{185.79,79.86,0,1},
{184.8,80.04,0,1},
{183.78,80.13,0,1},
{182.73,80.16,0,1},
{181.5,80.1,0,1},
{180.27,79.98,0,1},
{179.01,79.74,0,1},
{177.78,79.43999,0,1},
{176.55,79.02,0,1},
{175.29,78.54,0,1},
{174.06,77.93999,0,1},
{172.8,77.28,0,1},
{170.37,75.72,0,1},
{168.03,74.01,0,1},
{165.81,72.09,0,1},
{163.71,69.99,0,1},
{163.71,18.66,0,1},
{165.93,17.37,0,1},
{168,16.26,0,1},
{169.95,15.39,0,1},
{171.75,14.7,0,1},
{173.55,14.19,0,1},
{175.44,13.83,0,1},
{177.42,13.62,0,1},
{179.46,13.53,0,1},
{180.6,13.56,0,1},
{181.71,13.68,0,1},
{182.76,13.83,0,1},
{183.81,14.04,0,1},
{184.83,14.34,0,1},
{185.79,14.7,0,1},
{186.75,15.09,0,1},
{187.65,15.57,0,1},
{188.55,16.11,0,1},
{189.39,16.74,0,1},
{190.23,17.4,0,1},
{191.01,18.15,0,1},
{191.76,18.93,0,1},
{192.51,19.8,0,1},
{193.2,20.73,0,1},
{193.86,21.72,0,1},
{194.49,22.77,0,1},
{195.09,23.91,0,1},
{195.66,25.11,0,1},
{196.17,26.4,0,1},
{197.07,29.19,0,1},
{197.79,32.25,0,1},
{198.78,39.3,0,1},
{199.11,47.52,0,1},
{199.11,47.52,30,0},
{211.32,43.38,30,0},
{211.32,43.38,0,0},
{210.93,38.28,0,1},
{210.27,33.48,0,1},
{209.34,28.92,0,1},
{208.14,24.63,0,1},
{206.7,20.61,0,1},
{204.96,16.86,0,1},
{202.98,13.38,0,1},
{200.79,10.26,0,1},
{198.48,7.53,0,1},
{197.25,6.33,0,1},
{196.02,5.22,0,1},
{194.76,4.23,0,1},
{193.44,3.36,0,1},
{192.09,2.55,0,1},
{190.71,1.89,0,1},
{189.3,1.32,0,1},
{187.86,0.84,0,1},
{186.39,0.48,0,1},
{184.89,0.21,0,1},
{183.33,0.06,0,1},
{181.77,0,0,1},
{178.95,0.12,0,1},
{176.37,0.45,0,1},
{175.17,0.69,0,1},
{174,1.02,0,1},
{172.89,1.38,0,1},
{171.84,1.8,0,1},
{169.8,2.79,0,1},
{167.76,3.93,0,1},
{165.75,5.22,0,1},
{163.71,6.66,0,1},
{162.96,2.49,0,1},
{151.77,2.49,0,1},
{151.77,127.17,0,1},
{163.71,127.17,0,1},
{163.71,82.65,0,1},
{166.02,85.08,0,1},
{168.45,87.33,0,1},
{170.94,89.34,0,1},
{173.55,91.14,0,1},
{174.87,91.92,0,1},
{176.28,92.61,0,1},
{177.69,93.18,0,1},
{179.19,93.66,0,1},
{180.69,94.02,0,1},
{182.25,94.29,0,1},
{183.87,94.43999,0,1},
{185.52,94.5,0,1},
{186.99,94.43999,0,1},
{188.46,94.32,0,1},
{189.87,94.07999,0,1},
{191.22,93.75,0,1},
{192.54,93.3,0,1},
{193.83,92.79,0,1},
{195.09,92.16,0,1},
{196.29,91.47,0,1},
{197.46,90.66,0,1},
{198.57,89.75999,0,1},
{199.68,88.77,0,1},
{200.73,87.66,0,1},
{201.72,86.49,0,1},
{202.68,85.2,0,1},
{203.61,83.82,0,1},
{204.51,82.35,0,1},
{206.13,79.17,0,1},
{207.54,75.68999,0,1},
{208.74,71.93999,0,1},
{209.73,67.89,0,1},
{210.48,63.54,0,1},
{211.02,58.89,0,1},
{211.35,53.97,0,1},
{211.44,48.72,0,1},
{211.44,48.72,30,0},
{282.03,41.73,30,0},
{282.03,41.73,0,0},
{281.61,36.54,0,1},
{280.95,31.71,0,1},
{279.99,27.18,0,1},
{278.79,22.98,0,1},
{277.29,19.11,0,1},
{276.45,17.28,0,1},
{275.52,15.54,0,1},
{274.56,13.89,0,1},
{273.51,12.3,0,1},
{272.4,10.8,0,1},
{271.26,9.42,0,1},
{270.06,8.13,0,1},
{268.8,6.93,0,1},
{267.54,5.82,0,1},
{266.19,4.8,0,1},
{264.84,3.9,0,1},
{263.4,3.09,0,1},
{261.93,2.34,0,1},
{260.43,1.74,0,1},
{258.87,1.2,0,1},
{257.28,0.78,0,1},
{255.63,0.42,0,1},
{253.95,0.18,0,1},
{252.21,0.06,0,1},
{250.44,0,0,1},
{248.61,0.06,0,1},
{246.84,0.21,0,1},
{245.1,0.45,0,1},
{243.42,0.78,0,1},
{241.8,1.23,0,1},
{240.21,1.8,0,1},
{238.68,2.43,0,1},
{237.18,3.18,0,1},
{235.77,4.02,0,1},
{234.39,4.95,0,1},
{233.04,6,0,1},
{231.75,7.14,0,1},
{230.52,8.4,0,1},
{229.32,9.719999,0,1},
{228.21,11.16,0,1},
{227.1,12.69,0,1},
{225.12,16.02,0,1},
{223.38,19.62,0,1},
{221.94,23.52,0,1},
{220.74,27.69,0,1},
{219.81,32.13,0,1},
{219.15,36.87,0,1},
{218.73,41.91,0,1},
{218.61,47.22,0,1},
{218.73,52.65,0,1},
{219.15,57.81,0,1},
{219.84,62.64,0,1},
{220.8,67.14,0,1},
{222.03,71.34,0,1},
{223.53,75.24,0,1},
{225.3,78.81,0,1},
{227.37,82.08,0,1},
{228.48,83.58,0,1},
{229.65,84.99,0,1},
{230.85,86.31,0,1},
{232.08,87.50999,0,1},
{233.37,88.62,0,1},
{234.72,89.64,0,1},
{236.1,90.57,0,1},
{237.51,91.38,0,1},
{238.98,92.13,0,1},
{240.48,92.75999,0,1},
{242.04,93.27,0,1},
{243.63,93.72,0,1},
{245.28,94.05,0,1},
{246.96,94.29,0,1},
{248.67,94.43999,0,1},
{250.44,94.5,0,1},
{252.21,94.43999,0,1},
{253.95,94.29,0,1},
{255.63,94.05,0,1},
{257.28,93.72,0,1},
{258.87,93.3,0,1},
{260.43,92.75999,0,1},
{261.93,92.13,0,1},
{263.4,91.41,0,1},
{264.84,90.57,0,1},
{266.19,89.67,0,1},
{267.54,88.65,0,1},
{268.8,87.54,0,1},
{270.06,86.31,0,1},
{271.26,85.02,0,1},
{272.4,83.61,0,1},
{273.51,82.11,0,1},
{275.52,78.87,0,1},
{277.29,75.3,0,1},
{278.79,71.4,0,1},
{279.99,67.2,0,1},
{280.95,62.67,0,1},
{281.61,57.84,0,1},
{282.03,52.68,0,1},
{282.15,47.22,0,1},
{282.15,47.22,30,0},
{269.49,55.35,30,0},
{269.49,55.35,0,0},
{268.56,62.37,0,1},
{267.84,65.46,0,1},
{266.97,68.28,0,1},
{265.95,70.83,0,1},
{264.78,73.08,0,1},
{264.12,74.1,0,1},
{263.46,75.06,0,1},
{262.74,75.96,0,1},
{261.99,76.77,0,1},
{261.21,77.55,0,1},
{260.4,78.24,0,1},
{259.56,78.87,0,1},
{258.69,79.41,0,1},
{257.76,79.92,0,1},
{256.83,80.34,0,1},
{255.84,80.7,0,1},
{254.82,81,0,1},
{253.77,81.24,0,1},
{252.72,81.42,0,1},
{251.58,81.50999,0,1},
{250.44,81.54,0,1},
{249.3,81.50999,0,1},
{248.19,81.42,0,1},
{247.08,81.24,0,1},
{246.03,81,0,1},
{245.01,80.7,0,1},
{244.02,80.34,0,1},
{243.09,79.92,0,1},
{242.16,79.41,0,1},
{241.29,78.87,0,1},
{240.42,78.24,0,1},
{239.61,77.55,0,1},
{238.83,76.77,0,1},
{238.08,75.96,0,1},
{237.36,75.06,0,1},
{236.67,74.1,0,1},
{236.04,73.08,0,1},
{234.84,70.83,0,1},
{233.82,68.28,0,1},
{232.95,65.46,0,1},
{232.23,62.37,0,1},
{231.27,55.35,0,1},
{230.97,47.22,0,1},
{231.27,39.3,0,1},
{232.23,32.4,0,1},
{232.95,29.34,0,1},
{233.82,26.52,0,1},
{234.84,23.97,0,1},
{236.04,21.66,0,1},
{236.67,20.61,0,1},
{237.36,19.62,0,1},
{238.08,18.69,0,1},
{238.83,17.85,0,1},
{239.61,17.07,0,1},
{240.42,16.35,0,1},
{241.29,15.72,0,1},
{242.16,15.12,0,1},
{243.09,14.61,0,1},
{244.02,14.19,0,1},
{245.01,13.8,0,1},
{246.03,13.5,0,1},
{247.08,13.26,0,1},
{248.19,13.08,0,1},
{249.3,12.99,0,1},
{250.44,12.96,0,1},
{251.58,12.99,0,1},
{252.69,13.08,0,1},
{253.77,13.26,0,1},
{254.79,13.5,0,1},
{255.81,13.8,0,1},
{256.8,14.16,0,1},
{257.73,14.58,0,1},
{258.63,15.09,0,1},
{259.53,15.66,0,1},
{260.37,16.29,0,1},
{261.18,17.01,0,1},
{261.96,17.76,0,1},
{262.71,18.6,0,1},
{263.4,19.5,0,1},
{264.09,20.46,0,1},
{264.75,21.51,0,1},
{265.92,23.76,0,1},
{266.97,26.31,0,1},
{267.84,29.13,0,1},
{268.53,32.19,0,1},
{269.49,39.15,0,1},
{269.82,47.22,0,1},
{269.82,47.22,30,0},
{313.59,0.81,30,0},
{313.59,0.81,0,0},
{311.34,1.14,0,1},
{310.26,1.41,0,1},
{309.24,1.71,0,1},
{308.22,2.04,0,1},
{307.26,2.46,0,1},
{306.3,2.94,0,1},
{305.4,3.45,0,1},
{304.53,4.02,0,1},
{303.69,4.65,0,1},
{302.88,5.34,0,1},
{302.13,6.09,0,1},
{301.38,6.87,0,1},
{300.69,7.71,0,1},
{300,8.639999,0,1},
{299.4,9.599999,0,1},
{298.8,10.65,0,1},
{298.29,11.73,0,1},
{297.33,14.13,0,1},
{296.55,16.77,0,1},
{295.95,19.68,0,1},
{295.53,22.83,0,1},
{295.29,26.22,0,1},
{295.2,29.88,0,1},
{295.2,79.5,0,1},
{287.07,79.5,0,1},
{287.07,92.00999,0,1},
{295.2,92.00999,0,1},
{295.2,117.75,0,1},
{307.17,117.75,0,1},
{307.17,92.00999,0,1},
{329.25,92.00999,0,1},
{329.25,79.5,0,1},
{307.17,79.5,0,1},
{307.17,36.93,0,1},
{307.2,30.9,0,1},
{307.35,26.43,0,1},
{307.53,24.63,0,1},
{307.83,22.86,0,1},
{308.25,21.18,0,1},
{308.82,19.56,0,1},
{309.12,18.87,0,1},
{309.45,18.24,0,1},
{309.81,17.64,0,1},
{310.23,17.1,0,1},
{310.71,16.59,0,1},
{311.19,16.14,0,1},
{311.76,15.72,0,1},
{312.33,15.33,0,1},
{312.96,15,0,1},
{313.65,14.73,0,1},
{314.4,14.49,0,1},
{315.18,14.28,0,1},
{316.05,14.13,0,1},
{316.95,14.04,0,1},
{317.91,13.95,0,1},
{318.9,13.95,0,1},
{320.37,14.01,0,1},
{321.81,14.22,0,1},
{323.22,14.55,0,1},
{324.6,15.03,0,1},
{326.97,16.02,0,1},
{328.56,16.83,0,1},
{329.25,16.83,0,1},
{329.25,3.27,0,1},
{325.86,2.22,0,1},
{322.41,1.38,0,1},
{319.05,0.87,0,1},
{315.99,0.72,0,1},


        };

    }
}