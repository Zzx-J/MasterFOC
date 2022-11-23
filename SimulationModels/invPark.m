function [alpha,beta] = invPark(d,q,theta)

alpha = d*cos(theta)-q*sin(theta);
beta = d*sin(theta)+q*cos(theta);