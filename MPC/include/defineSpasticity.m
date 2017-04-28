% This function takes a subject's score on the Modified Ashworth scale and
% outputs arm parameters related to tone and spasticity (gamma = static
% reflex threshold, mu = slope of line defining dynamic reflex threshold, k
% = reflex stiffness, b = reflex damping).
function [gamma, mu, k, b] = defineSpasticity(MAS, data)

% initialize tone/spasticity parameters
nJoints = size(data,3);
gamma = zeros(2*nJoints,1);
mu = zeros(2*nJoints,1);
k = zeros(2*nJoints,1);
b = zeros(2*nJoints,1);

% assume same spastic response for each joint in +/- (e.g., flex/ext)
fitsFull = zeros(4,2,2*nJoints);      % joint +/- data
indMap = sort(repmat(1:nJoints,1,2)); % index map from joint data to joint +/- data
for i = 1:2*nJoints
    fitsFull(:,:,i) = data(:,:,indMap(i));
end
data = fitsFull;

% update parameters based on Modified Ashworth scale
for i = 1:2*nJoints
    gamma(i) = data(1,1,i)*MAS + data(1,2,i);
    mu(i) = data(2,1,i)*MAS + data(2,2,i);
    k(i) = data(3,1,i)*MAS + data(3,2,i);
    b(i) = data(4,1,i)*MAS + data(4,2,i);
end

end