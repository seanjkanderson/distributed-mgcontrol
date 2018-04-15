function xhat = ufk_own(u_k,z_k,params)

persistent obj;

if isempty(obj)
    initialState = 310;
    obj = unscentedKalmanFilter(@vdpStateFcn,@vdpMeasurementFcn,...
        initialState, 'HasAdditiveMeasurementNoise',false);
    obj.MeasurementNoise = 0.01;
end
% Estimate the states.
CorrectedX = correct(obj,output);
predict(obj);
end

