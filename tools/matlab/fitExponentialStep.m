% x and y should be column vectors; m x 1
% f(x) = a * (1-exp(b*(c-x)))
function [a, b, c] = fitExponentialStep(x, y, varargin)
    coeffs0 = [max(y), 1, 0]; % initialization point
    model = @(coeff,t)(coeff(1) * (1-exp(coeff(2) * (coeff(3)-t(:, 1)))));
    
    if (length(varargin) > 0)
        weights = varargin{1};        
        coeffs = fitNonlinearModel(x, y, model, coeffs0, 'weights', weights);
    else
        coeffs = fitNonlinearModel(x, y, model, coeffs0);
    end
    
    a = coeffs(1);
    b = coeffs(2);
    c = coeffs(3);   