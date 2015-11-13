function GMp = Generate_1st_Order_Markov(WhiteNoise, Tc)
    beta = 1/Tc;    % Degree of correlation
    [nbSequences, nbSamples] = size(WhiteNoise);
    GMp = zeros(nbSequences, nbSamples);
    for i = 2:nbSamples
        GMp(:,i) = exp(-beta)*GMp(:,i-1)+WhiteNoise(:,i);
    end
end

