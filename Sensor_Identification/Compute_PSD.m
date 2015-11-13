function PSD = Compute_PSD(Signals)
    [nbSequences, ~] = size(Signals);
    Hs = spectrum.welch;

    for i = 1:nbSequences
        PSD(i) = msspectrum(Hs, Signals(1,:), 'Fs', 20);
    end
end

