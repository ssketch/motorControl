% This function takes joint-wise bias data (joint angle vs. bias at that
% angle) and outputs the affine function fit (slope and intercept) for each
% joint.
function biasFunc = defineBiasFunc(data)

for i = 1:size(data,2)
    [biasFunc.slope(i,:), biasFunc.inter(i,:)] = ...
        fitData(data.angle(:,i), data.bias(:,i));
end

end