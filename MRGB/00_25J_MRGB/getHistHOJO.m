function normHist = getHist(val, bins, continueous, shareWeight)
if nargin<4
    shareWeight = false;
end

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
    if ~shareWeight
    % old method - 1 bin only
        [D,I] = min(pdist2(val,bins','cosine'));
        normHist(I) = 1; %old - 1 bin only
    else
        % share with 2 bins
        [D,I] = mink(pdist2(val,bins','cosine'),2);
    
        ttl = sum(D);
        normHist(I(1)) = D(2) / ttl;    % normalize
        normHist(I(2)) = D(1) / ttl;    % normalize
    end
end
end