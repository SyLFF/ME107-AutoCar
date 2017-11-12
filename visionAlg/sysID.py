"""
function [A,B] = SIDcalc(X,U,nX,nU,nsamples)

	% Note the following about the state vector (X) and input vector (U)

	% The dimensions of the state vector should be (nX*nSamples,1);

	% The dimensions of the input vector should be (nU*(nSamples-1),1);

	% Will set this up as a least squares problem (as that's what it is..)

	b = X(nX+1:end);

	A = [];

	for i = 1:nsamples

		A = [A;kron(eye(nX),X(nX*(i-1)+1:nX*i)'), kron(eye(nX),U(nU*(i-1)+1:nU*i)')];

	end

	coeff = A\b;

	A = reshape(coeff(1:nX^2),[nX,nX])';

	B = reshape(coeff(nX^2+1:end),[nU,nX])';

end 
"""

"""
%% Clear Workspace
clear, clc
%% Load Variables
u(:,1) = struct2array(load('u12v20.mat'));
time(:,1) = struct2array(load('time12v20.mat'));
theta(:,1) = struct2array(load('theta12v20.mat'));
%% Write Struct
u = u(2:length(u));
time = time(2:length(time));
theta = theta(2:length(theta));
time = time - time(1);
data.u = u;
data.t = time/1000;
data.x1 = theta;
Ts = data.t(2) - data.t(1);
data.x2 = diff(data.x1)./diff(data.t);
data.u = data.u(1:length(data.u)-1);
data.t = data.t(1:length(data.t)-1);
data.x1 = data.x1(1:length(data.x1)-1);
%% Test System ID code (Comment out when running actual ID)
% clear, clc
% Ar = [-0.2 1; 0 -0.8];
% Arvec = [Ar(1:2),Ar(3:4)]';
% Br = [0; 5];
% Arvec = [Arvec;Br];
% u = 24*rand(120,1)-12;
% x = zeros(120,2);
% x(1,:) = [0 0];
% for i = 2:numel(u)
%     x(i,:) = Ar*x(i-1,:)'+Br*u(i-1)+1*randn;
% end
% data.u = u;
% data.x1 = x(:,1);
% data.x2 = x(:,2);
%% System ID
X = zeros(2*(numel(data.u)-1),6);
Y = zeros(2*(numel(data.u)-1),1);
for i = 1:2*(numel(data.u)-1)
    n = ceil(i/2);
    if rem(i,2) == 0 % even case
        X(i,:) = [0 data.x1(n) 0 data.x2(n) 0 data.u(n)];
        Y(i) = data.x2(n+1);
    else % odd case
        X(i,:) = [data.x1(n) 0 data.x2(n) 0 data.u(n) 0];
        Y(i) = data.x1(n+1);
    end
end
Avec = linsolve (X,Y);
Avec2 = X\Y;
A = reshape(Avec(1:4),[2,2]);
B = Avec(5:6);
%% Test System ID code (Comment out when running actual ID)
% for i = 1:numel(X(:,1))
%     true(i) = X(i,:)*Arvec ~= Y(i);
% end
% works = sum(true);
% [A Ar]
% [B Br]
"""

import numpy as np

def sysID(X, U, nX, nU, nSamples):
	b = X(nX + 1:end)
	A = []

	for i in range(0, nSamples):
		A = [(A), 
			(kron(identity(nX), X(nX*(i - 1) + 1:nX*i).transpose), kron(identity(nX), U(nU*(i-1)+1:nU*i).transpose))]
	coeff = divide(A, b)

	A = reshape(coeff(1:nX^2), [nX, nX]).transpose
	B = reshape(coeff(nX^2 + 1: end),[nU,nX]).transpose

y = # distance from front of the car to bottom of the image + distance from bottom of the image to centroid
e = # horizontal deviation from geometric center
alpha = #arctan(y / e)

