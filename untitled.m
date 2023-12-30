
% 测试示例
x0 = 0; y0 = 0; z0 = 0;
x1 = 1; y1 = 1; z1 = 1;
planeVal = 0.5;
planeAxis = 'x';

intersection = intersectionWithPlane(x0, y0, z0, x1, y1, z1, planeVal, planeAxis)
function intersection = intersectionWithPlane(x0, y0, z0, x1, y1, z1, planeVal, planeAxis)
    t = (planeVal - eval([planeAxis '0'])) / (eval([planeAxis '1']) - eval([planeAxis '0']));
    intersection = [x0 + t*(x1 - x0), y0 + t*(y1 - y0), z0 + t*(z1 - z0)];
end
