% This function takes a subject's score on the Modified Ashworth scale and
% updates arm parameters related to tone and spasticity (gamma = static
% reflex threshold, mu = slope of line defining dynamic reflex threshold, k
% = reflex stiffness, b = reflex damping).
function defineSpasticity(arm, MAS, data)

nJoints = length(arm.q.val);

% assume same spastic response for each joint in +/- (e.g., flex/ext)
if size(data,3) == nJoints
    fitsFull = zeros(4,2,2*nJoints);      % joint +/- data
    indMap = sort(repmat(1:nJoints,1,2)); % index map from joint data to joint +/- data
    for i = 1:2*nJoints
        fitsFull(:,:,i) = data(:,:,indMap(i));
    end
    data = fitsFull;
end

% check for mismatch between amount of data and number of joints (need +/-
% in each DOF)
if size(data,3) ~= 2*nJoints
    error('Mismatch between amount of data and arm DOFs.')
end

% update arm parameters related to tone/spasticity
for i = 1:2*nJoints
    arm.gamma(i) = data(1,1,i)*MAS + data(1,2,i);
    arm.mu(i) = data(2,1,i)*MAS + data(2,2,i);
    arm.k(i) = data(3,1,i)*MAS + data(3,2,i);
    arm.b(i) = data(4,1,i)*MAS + data(4,2,i);
end

end