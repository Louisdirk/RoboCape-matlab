function RandomWalk = Generate_Random_Walk(WhiteNoise)
    [nbSequences, nbSamples] = size(WhiteNoise);
    RandomWalk = zeros(nbSequences, nbSamples);
    for i = 2:nbSamples
       RandomWalk(:,i) = RandomWalk(:,i-1)+WhiteNoise(:,i); 
    end
end

