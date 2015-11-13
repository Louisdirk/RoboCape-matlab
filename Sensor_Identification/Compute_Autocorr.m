function AutoCorr = Compute_Autocorr(Signals)
    [nbSequences, nbSamples] = size(Signals);
    AutoCorr = zeros(nbSequences, 2*nbSamples-1);
    for i = 1:nbSequences
        AutoCorr(i,:) = var(Signals(i,:))*xcorr(Signals(i,:), 'coeff');
    end
end

