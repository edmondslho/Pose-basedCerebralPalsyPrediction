function normHist = getHist(val, bins, continueous)
numBin = length(bins);
normHist = zeros(1,numBin);

if ~continueous
    % find the best bin
    for cc=1:numBin
        if (val < bins(cc))
            normHist(cc) = 1;
            break;  % early exit
        end
    end
else    
    [D,I] = min(pdist2(val,bins','cosine'));
    normHist(I) = 1;
end
end