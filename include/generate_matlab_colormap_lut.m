clc;
clear all;

numDivisions = 256;

lut = colormap(jet(numDivisions)) * (numDivisions-1);

for i=1:numDivisions
    if i < numDivisions
        disp(sprintf('{%d,%d,%d},', lut(i, 1), lut(i, 2), lut(i, 3)));
    else
        disp(sprintf('{%d,%d,%d}', lut(i, 1), lut(i, 2), lut(i, 3)));
    end
end