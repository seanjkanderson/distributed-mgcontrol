% MPC solver for quasi-real-time
function [out1, out2, out3, out4] = mpc_run(inputs)

persistent i_count

if isempty(i_count)
    run mpc_init
    i_count = 0;
    inputs = sqrt(2/3) * [4160, 4000, 3900, 4200]';
else
    i_count = i_count + 1;
end

outputs = PPC_tilde*inputs;
out1 = outputs
